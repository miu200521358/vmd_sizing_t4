package usecase

import (
	"fmt"
	"miu200521358/vmd_sizing_t4/pkg/domain"
	"runtime"
	"sync"
	"sync/atomic"
	"time"

	"github.com/miu200521358/mlib_go/pkg/config/merr"
	"github.com/miu200521358/mlib_go/pkg/config/mi18n"
	"github.com/miu200521358/mlib_go/pkg/config/mlog"
	"github.com/miu200521358/mlib_go/pkg/domain/delta"
	"github.com/miu200521358/mlib_go/pkg/domain/mmath"
	"github.com/miu200521358/mlib_go/pkg/domain/pmx"
	"github.com/miu200521358/mlib_go/pkg/domain/vmd"
	"github.com/miu200521358/mlib_go/pkg/infrastructure/mfile"
	"github.com/miu200521358/mlib_go/pkg/infrastructure/miter"
	"github.com/miu200521358/mlib_go/pkg/infrastructure/repository"
	"github.com/miu200521358/mlib_go/pkg/interface/controller"
	"github.com/miu200521358/mlib_go/pkg/usecase/deform"
)

func ExecSizing(cw *controller.ControlWindow, sizingState *domain.SizingState) {
	if !sizingState.AdoptSizingCheck.Checked() ||
		(sizingState.CurrentSet().OriginalModel == nil &&
			sizingState.CurrentSet().SizingConfigModel == nil &&
			sizingState.CurrentSet().OutputMotion == nil) {
		return
	}

	for _, sizingSet := range sizingState.SizingSets {
		if !sizingSet.IsSizingLeg && sizingSet.CompletedSizingLeg ||
			!sizingSet.IsSizingUpper && sizingSet.CompletedSizingUpper ||
			!sizingSet.IsSizingShoulder && sizingSet.CompletedSizingShoulder ||
			!sizingSet.IsSizingArmStance && sizingSet.CompletedSizingArmStance ||
			!sizingSet.IsSizingFingerStance && sizingSet.CompletedSizingFingerStance ||
			!sizingSet.IsSizingArmTwist && sizingSet.CompletedSizingArmTwist ||
			!sizingSet.IsSizingWrist && sizingSet.CompletedSizingWrist ||
			sizingSet.ShoulderWeight != sizingSet.CompletedShoulderWeight {

			// チェックを外したら読み直し
			sizingSet.CompletedSizingLeg = false
			sizingSet.CompletedSizingUpper = false
			sizingSet.CompletedSizingShoulder = false
			sizingSet.CompletedSizingArmStance = false
			sizingSet.CompletedSizingFingerStance = false
			sizingSet.CompletedSizingArmTwist = false
			sizingSet.CompletedSizingWrist = false

			// オリジナルモーションをサイジング先モーションとして読み直し
			sizingState.SetCurrentIndex(sizingSet.Index)
			sizingState.LoadSizingMotion(cw, sizingSet.OriginalMotionPath)
		}
	}

	var completedProcessCount int32 = 0
	totalProcessCount := 0
	for _, sizingSet := range sizingState.SizingSets {
		totalProcessCount += sizingSet.GetProcessCount()
	}

	cw.Synchronize(func() {
		sizingState.SetSizingEnabled(false)
		sizingState.TerminateButton.SetEnabled(true)

		cw.ProgressBar().SetMax(totalProcessCount)
		cw.ProgressBar().SetValue(int(completedProcessCount))
	})

	// 処理時間の計測開始
	start := time.Now()

	scales := generateSizingScales(sizingState.SizingSets)
	isExec := false

	errorChan := make(chan error, len(sizingState.SizingSets))

	mlog.IL(mi18n.T("サイジング開始"))

	var wg sync.WaitGroup
	for _, sizingSet := range sizingState.SizingSets {
		if sizingSet.OriginalConfigModel == nil || sizingSet.SizingConfigModel == nil ||
			sizingSet.OutputMotion == nil {
			continue
		}

		wg.Add(1)
		go func(sizingSet *domain.SizingSet) {
			defer wg.Done()

			incrementCompletedCount := func() {
				atomic.AddInt32(&completedProcessCount, 1)
				cw.Synchronize(func() {
					cw.ProgressBar().Increment()
				})
			}

			// 腕指スタンス補正
			if execResult, err := SizingArmFingerStance(sizingSet, len(sizingState.SizingSets), incrementCompletedCount); err != nil {
				errorChan <- err
				return
			} else {
				if sizingSet.IsTerminate {
					isExec = false
					return
				}

				isExec = execResult || isExec

				if isExec {
					sizingSet.OutputMotion.SetRandHash()
					sizingSet.OutputMotion.SetName(sizingSet.SizingModel.Name())
					cw.StoreMotion(0, sizingSet.Index, sizingSet.OutputMotion)
				}
			}

			// 下半身・足補正
			if execResult, err := SizingLeg(sizingSet, scales[sizingSet.Index], len(sizingState.SizingSets), incrementCompletedCount); err != nil {
				errorChan <- err
				return
			} else {
				if sizingSet.IsTerminate {
					isExec = false
					return
				}

				isExec = execResult || isExec

				if isExec {
					sizingSet.OutputMotion.SetRandHash()
					sizingSet.OutputMotion.SetName(sizingSet.SizingModel.Name())
					cw.StoreMotion(0, sizingSet.Index, sizingSet.OutputMotion)
				}
			}

			for _, funcUsecase := range []func(sizingSet *domain.SizingSet, sizingSetCount int, incrementCompletedCount func()) (bool, error){
				SizingUpper,    // 上半身補正
				SizingShoulder, // 肩補正
				SizingWrist,    // 手首位置合わせ
			} {
				if execResult, err := funcUsecase(sizingSet, len(sizingState.SizingSets), incrementCompletedCount); err != nil {
					errorChan <- err
					return
				} else {
					if sizingSet.IsTerminate {
						isExec = false
						return
					}

					isExec = execResult || isExec

					if isExec {
						sizingSet.OutputMotion.SetRandHash()
						sizingSet.OutputMotion.SetName(sizingSet.SizingModel.Name())
						cw.StoreMotion(0, sizingSet.Index, sizingSet.OutputMotion)
					}
				}
			}

		}(sizingSet)
	}

	wg.Wait()
	close(errorChan)

	// チャネルからエラーを受け取る
	for err := range errorChan {
		if err != nil {
			if err == merr.TerminateError {
				mlog.I(mi18n.T("サイジング中断"))
				break
			} else {
				mlog.E(mi18n.T("サイジングエラー", map[string]interface{}{
					"Error": err.Error(), "AppName": cw.AppConfig().Name, "AppVersion": cw.AppConfig().Version}))
				panic(err)
			}
		}
	}

	// 処理時間の計測終了
	elapsed := time.Since(start)

	if isExec {
		mlog.ILT(mi18n.T("サイジング終了"), mi18n.T("サイジング終了メッセージ",
			map[string]interface{}{"ProcessTime": controller.FormatDuration(elapsed)}))
	} else {
		mlog.I(mi18n.T("サイジング終了"))
	}

	// 中断したら、データを戻してフラグを落としておく
	for _, sizingSet := range sizingState.SizingSets {
		if sizingSet.IsTerminate {
			// オリジナルモーションをサイジング先モーションとして読み直し
			outputMotion := cw.LoadMotion(1, sizingSet.Index)
			outputMotion.SetRandHash()
			cw.StoreMotion(0, sizingSet.Index, outputMotion)

			sizingSet.IsTerminate = false
		}
	}

	cw.Synchronize(func() {
		sizingState.SetSizingEnabled(true)
		sizingState.TerminateButton.SetEnabled(false)

		cw.ProgressBar().SetMax(0)
		cw.ProgressBar().SetValue(0)
	})

	// 最初に戻す(読み直しとかでINDEXがズレた時用)
	sizingState.ChangeCurrentAction(0)
	controller.Beep()
}

func generateSizingScales(sizingSets []*domain.SizingSet) (scales []*mmath.MVec3) {
	scales = make([]*mmath.MVec3, len(sizingSets))

	// 複数人居るときはXZは共通のスケールを使用する
	meanXZScale := 0.0

	for i, sizingSet := range sizingSets {
		originalModel := sizingSet.OriginalModel
		sizingModel := sizingSet.SizingConfigModel

		if originalModel == nil || sizingModel == nil {
			scales[i] = &mmath.MVec3{X: 1.0, Y: 1.0, Z: 1.0}
			meanXZScale += 1.0
			continue
		}

		sizingNeckRoot, _ := sizingModel.Bones.GetNeckRoot()
		sizingLeftLeg, _ := sizingModel.Bones.GetLeg(pmx.BONE_DIRECTION_LEFT)
		sizingLeftKnee, _ := sizingModel.Bones.GetKnee(pmx.BONE_DIRECTION_LEFT)
		sizingLeftAnkle, _ := sizingModel.Bones.GetAnkle(pmx.BONE_DIRECTION_LEFT)
		sizingLeftLegIK, _ := sizingModel.Bones.GetLegIk(pmx.BONE_DIRECTION_LEFT)
		sizingTrunkRoot, _ := sizingModel.Bones.GetTrunkRoot()
		originalNeckRoot, _ := originalModel.Bones.GetNeckRoot()
		originalLeftLeg, _ := originalModel.Bones.GetLeg(pmx.BONE_DIRECTION_LEFT)
		originalLeftKnee, _ := originalModel.Bones.GetKnee(pmx.BONE_DIRECTION_LEFT)
		originalLeftAnkle, _ := originalModel.Bones.GetAnkle(pmx.BONE_DIRECTION_LEFT)
		originalLeftLegIK, _ := originalModel.Bones.GetLegIk(pmx.BONE_DIRECTION_LEFT)
		originalTrunkRoot, _ := originalModel.Bones.GetTrunkRoot()

		if sizingLeftLeg == nil || sizingLeftKnee == nil || sizingLeftAnkle == nil || sizingLeftLegIK == nil ||
			originalLeftLeg == nil || originalLeftKnee == nil || originalLeftAnkle == nil || originalLeftLegIK == nil ||
			sizingTrunkRoot == nil || originalTrunkRoot == nil {
			if sizingNeckRoot != nil && originalNeckRoot != nil {
				// 首根元までの長さ比率
				neckLengthRatio := sizingNeckRoot.Position.Y / originalNeckRoot.Position.Y
				scales[i] = &mmath.MVec3{X: neckLengthRatio, Y: neckLengthRatio, Z: neckLengthRatio}
				meanXZScale += neckLengthRatio
			} else {
				scales[i] = &mmath.MVec3{X: 1.0, Y: 1.0, Z: 1.0}
				meanXZScale += 1.0
			}
		} else {
			// 足の長さ比率(XZ)
			legLengthRatio := (sizingLeftLeg.Position.Distance(sizingLeftKnee.Position) +
				sizingLeftKnee.Position.Distance(sizingLeftAnkle.Position)) /
				(originalLeftLeg.Position.Distance(originalLeftKnee.Position) +
					originalLeftKnee.Position.Distance(originalLeftAnkle.Position))
			// 体幹中心までの長さ比率
			legCenterRatio := sizingTrunkRoot.Position.Y / originalTrunkRoot.Position.Y

			// // 足の長さ比率(Y)
			// legHeightRatio := sizingLeftLeg.Position.Distance(sizingLeftAnkle.Position) /
			// 	originalLeftLeg.Position.Distance(originalLeftAnkle.Position)

			scales[i] = &mmath.MVec3{X: legLengthRatio, Y: legCenterRatio, Z: legLengthRatio}
			meanXZScale += legLengthRatio
		}
	}

	// 複数人いるときはXZは共通のスケールを使用する
	meanXZScale /= float64(len(scales))
	newXZScale := meanXZScale
	if len(sizingSets) > 1 {
		newXZScale = min(1.2, meanXZScale)
	}

	for i, sizingSet := range sizingSets {
		if sizingSet.IsSizingLeg && !sizingSet.CompletedSizingLeg {
			mlog.I(mi18n.T("移動補正スケール", map[string]interface{}{
				"No": i + 1, "XZ": fmt.Sprintf("%.3f", newXZScale),
				"OrgXZ": fmt.Sprintf("%.3f", scales[i].X), "Y": fmt.Sprintf("%.3f", scales[i].Y)}))
		}

		scales[i].X = newXZScale
		scales[i].Z = newXZScale
	}

	return scales
}

// getFrames 処理対象のフレームを取得する
func getFrames(motion *vmd.VmdMotion, boneNames []string) []int {
	frames := motion.BoneFrames.IndexesByNames(boneNames)
	// TODO
	// rangeFrames := mmath.IntRangesByStep(0, int(motion.MaxFrame()), 5)
	// frames = append(frames, rangeFrames...)

	// frames = mmath.Unique(frames)
	// mmath.Sort(frames)

	return frames
}

// processLog 処理ログを出力する
func processLog(key string, index, iterIndex, allCount int) {
	mlog.I(mi18n.T(key, map[string]interface{}{
		"No":        index + 1,
		"IterIndex": fmt.Sprintf("%04d", iterIndex),
		"AllCount":  fmt.Sprintf("%02d", allCount),
	}))
}

func outputVerboseMotion(title string, motionPath string, motion *vmd.VmdMotion) {
	outputPath := mfile.CreateOutputPath(motionPath, title)
	repository.NewVmdRepository(true).Save(outputPath, motion, true)
	mlog.V("%s: %s", title, outputPath)
}

// computeVmdDeltas は、各フレームごとのデフォーム結果を並列処理で取得します。
func computeVmdDeltas(
	frames []int, blockSize int,
	model *pmx.PmxModel, motion *vmd.VmdMotion,
	sizingSet *domain.SizingSet,
	isCalcIk bool, target_bone_names []string, logKey string,
) ([]*delta.VmdDeltas, error) {
	allDeltas := make([]*delta.VmdDeltas, len(frames))
	err := miter.IterParallelByList(frames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			frame := float32(data)
			vmdDeltas := delta.NewVmdDeltas(frame, model.Bones, model.Hash(), motion.Hash())
			vmdDeltas.Morphs = deform.DeformBoneMorph(model, motion.MorphFrames, frame, nil)
			vmdDeltas.Bones = deform.DeformBone(model, motion, isCalcIk, data, target_bone_names)
			allDeltas[index] = vmdDeltas
			return nil
		},
		func(iterIndex, allCount int) {
			if logKey != "" {
				processLog(logKey, sizingSet.Index, iterIndex, allCount)
			}
		})
	return allDeltas, err
}

// ログはCPUのサイズに応じて可変でブロッキングして出力する
var log_block_size = runtime.NumCPU() * 50

// 方向
var directions = []pmx.BoneDirection{pmx.BONE_DIRECTION_LEFT, pmx.BONE_DIRECTION_RIGHT}

// 体幹下部ボーン名
var trunk_lower_bone_names = []string{
	pmx.ROOT.String(), pmx.TRUNK_ROOT.String(), pmx.CENTER.String(), pmx.GROOVE.String(), pmx.WAIST.String(),
	pmx.LOWER_ROOT.String(), pmx.LOWER.String(), pmx.LEG_CENTER.String(), pmx.LEG.Left(), pmx.LEG.Right()}

// 足関連ボーン名（左右別）
var leg_direction_bone_names = [][]string{
	{pmx.LEG.Left(), pmx.KNEE.Left(), pmx.HEEL.Left(), pmx.ANKLE.Left(), pmx.TOE_T.Left(), pmx.TOE_P.Left(),
		pmx.TOE_C.Left(), pmx.LEG_D.Left(), pmx.KNEE_D.Left(), pmx.HEEL_D.Left(), pmx.ANKLE_D.Left(),
		pmx.TOE_T_D.Left(), pmx.TOE_P_D.Left(), pmx.TOE_C_D.Left(), pmx.TOE_EX.Left(),
		pmx.LEG_IK_PARENT.Left(), pmx.LEG_IK.Left(), pmx.TOE_IK.Left()},
	{pmx.LEG.Right(), pmx.KNEE.Right(), pmx.HEEL.Right(), pmx.ANKLE.Right(), pmx.TOE_T.Right(), pmx.TOE_P.Right(),
		pmx.TOE_C.Right(), pmx.LEG_D.Right(), pmx.KNEE_D.Right(), pmx.HEEL_D.Right(), pmx.ANKLE_D.Right(),
		pmx.TOE_T_D.Right(), pmx.TOE_P_D.Right(), pmx.TOE_C_D.Right(), pmx.TOE_EX.Right(),
		pmx.LEG_IK_PARENT.Right(), pmx.LEG_IK.Right(), pmx.TOE_IK.Right()},
}

// 足関連ボーン名（両方向）
var leg_all_direction_bone_names = append(leg_direction_bone_names[0], leg_direction_bone_names[1]...)

// 全ての下半身ボーン名
var all_lower_leg_bone_names = append(trunk_lower_bone_names, leg_all_direction_bone_names...)

// 重心計算対象ボーン名（つま先とか指先は入っていない）
var gravity_bone_names = []string{
	pmx.HEAD.String(), pmx.NECK_ROOT.String(), pmx.ARM.Left(), pmx.ARM.Right(), pmx.ELBOW.Left(), pmx.ELBOW.Right(),
	pmx.WRIST.Left(), pmx.WRIST.Right(), pmx.UPPER_ROOT.String(), pmx.LOWER_ROOT.String(), pmx.LEG_CENTER.String(),
	pmx.LEG_D.Left(), pmx.LEG_D.Right(), pmx.KNEE_D.Left(), pmx.KNEE_D.Right(), pmx.HEEL_D.Left(), pmx.HEEL_D.Right(),
}

// 下半身系 + 重力計算対象ボーン名
var all_gravity_lower_leg_bone_names = append(all_lower_leg_bone_names, gravity_bone_names...)

// 上半身系ボーン名
var trunk_upper_bone_names = []string{
	pmx.ROOT.String(), pmx.TRUNK_ROOT.String(), pmx.CENTER.String(), pmx.GROOVE.String(), pmx.WAIST.String(),
	pmx.UPPER_ROOT.String(), pmx.UPPER.String(), pmx.UPPER2.String(), pmx.NECK_ROOT.String(),
	pmx.SHOULDER.Left(), pmx.SHOULDER.Right(), pmx.NECK.String()}

// 腕系ボーン名（左右別）
var all_arm_stance_bone_names = [][]string{
	{pmx.ARM.Left(), pmx.ELBOW.Left(), pmx.WRIST.Left(), pmx.WRIST_TAIL.Left(),
		pmx.THUMB0.Left(), pmx.THUMB1.Left(), pmx.THUMB2.Left(), pmx.THUMB_TAIL.Left(),
		pmx.INDEX1.Left(), pmx.INDEX2.Left(), pmx.INDEX3.Left(), pmx.INDEX_TAIL.Left(),
		pmx.MIDDLE1.Left(), pmx.MIDDLE2.Left(), pmx.MIDDLE3.Left(), pmx.MIDDLE_TAIL.Left(),
		pmx.RING1.Left(), pmx.RING2.Left(), pmx.RING3.Left(), pmx.RING_TAIL.Left(),
		pmx.PINKY1.Left(), pmx.PINKY2.Left(), pmx.PINKY3.Left(), pmx.PINKY_TAIL.Left()},
	{pmx.ARM.Right(), pmx.ELBOW.Right(), pmx.WRIST.Right(), pmx.WRIST_TAIL.Right(),
		pmx.THUMB0.Right(), pmx.THUMB1.Right(), pmx.THUMB2.Right(), pmx.THUMB_TAIL.Right(),
		pmx.INDEX1.Right(), pmx.INDEX2.Right(), pmx.INDEX3.Right(), pmx.INDEX_TAIL.Right(),
		pmx.MIDDLE1.Right(), pmx.MIDDLE2.Right(), pmx.MIDDLE3.Right(), pmx.MIDDLE_TAIL.Right(),
		pmx.RING1.Right(), pmx.RING2.Right(), pmx.RING3.Right(), pmx.RING_TAIL.Right(),
		pmx.PINKY1.Right(), pmx.PINKY2.Right(), pmx.PINKY3.Right(), pmx.PINKY_TAIL.Right()},
}

// 腕系ボーン名（左右別）
var all_arm_bone_names = [][]string{
	{pmx.TRUNK_ROOT.String(), pmx.NECK_ROOT.String(), pmx.SHOULDER.Left(),
		pmx.ARM.Left(), pmx.ELBOW.Left(), pmx.WRIST.Left(), pmx.WRIST_TAIL.Left()},
	{pmx.TRUNK_ROOT.String(), pmx.NECK_ROOT.String(), pmx.SHOULDER.Right(),
		pmx.ARM.Right(), pmx.ELBOW.Right(), pmx.WRIST.Right(), pmx.WRIST_TAIL.Right()},
}
