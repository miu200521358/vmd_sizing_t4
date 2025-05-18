package usecase

import (
	"fmt"
	"miu200521358/vmd_sizing_t4/pkg/domain"
	"runtime"

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
	"github.com/miu200521358/mlib_go/pkg/usecase/deform"
)

func GenerateSizingScales(sizingSets []*domain.SizingSet) (scales []*mmath.MVec3) {
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
			legLengthScale := (sizingLeftLeg.Position.Distance(sizingLeftKnee.Position) +
				sizingLeftKnee.Position.Distance(sizingLeftAnkle.Position)) /
				(originalLeftLeg.Position.Distance(originalLeftKnee.Position) +
					originalLeftKnee.Position.Distance(originalLeftAnkle.Position))
			// // 体幹中心までの長さ比率
			// trunkRootScale := sizingTrunkRoot.Position.Y / originalTrunkRoot.Position.Y

			scales[i] = &mmath.MVec3{X: legLengthScale, Y: legLengthScale, Z: legLengthScale}
			meanXZScale += legLengthScale
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
			mlog.I(mi18n.T("移動補正スケール", map[string]any{
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
	mlog.I(mi18n.T(key, map[string]any{
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
	incrementCompletedCount func(),
) ([]*delta.VmdDeltas, error) {
	allDeltas := make([]*delta.VmdDeltas, len(frames))
	err := miter.IterParallelByList(frames, blockSize, log_block_size,
		func(index, iFrame int) error {
			if sizingSet.IsTerminate {
				return merr.NewTerminateError("manual terminate")
			}

			allDeltas[index] = deform.DeformBone(model, motion, motion, isCalcIk, iFrame, target_bone_names)

			if incrementCompletedCount != nil {
				incrementCompletedCount()
			}

			return nil
		},
		func(iterIndex, allCount int) {
			if logKey != "" {
				processLog(logKey, sizingSet.Index, iterIndex, allCount)
			}
		})
	return allDeltas, err
}

// computeVmdDeltasWithDeltas は、各フレームごとのデフォーム結果を並列処理で取得します。
func computeVmdDeltasWithDeltas(
	frames []int, blockSize int,
	model *pmx.PmxModel, motion *vmd.VmdMotion, allDeltas []*delta.VmdDeltas,
	sizingSet *domain.SizingSet,
	isCalcIk bool, target_bone_names []string, logKey string,
	incrementCompletedCount func(),
) ([]*delta.VmdDeltas, error) {
	err := miter.IterParallelByList(frames, blockSize, log_block_size,
		func(index, iFrame int) error {
			if sizingSet.IsTerminate {
				return merr.NewTerminateError("manual terminate")
			}

			allDeltas[index] = deform.DeformBoneWithDeltas(model, motion, allDeltas[index], isCalcIk, iFrame, target_bone_names)

			if incrementCompletedCount != nil {
				incrementCompletedCount()
			}

			return nil
		},
		func(iterIndex, allCount int) {
			if logKey != "" {
				processLog(logKey, sizingSet.Index, iterIndex, allCount)
			}
		})
	return allDeltas, err
}

// computeVmdDeltas は、各フレームごとのボーンモーフだけのデフォーム結果を並列処理で取得します。
func computeMorphVmdDeltas(
	frames []int, blockSize int,
	model *pmx.PmxModel, motion *vmd.VmdMotion,
	sizingSet *domain.SizingSet, target_bone_names []string, logKey string,
	incrementCompletedCount func(),
) ([]*delta.VmdDeltas, error) {
	allDeltas := make([]*delta.VmdDeltas, len(frames))
	err := miter.IterParallelByList(frames, blockSize, log_block_size,
		func(index, iFrame int) error {
			if sizingSet.IsTerminate {
				return merr.NewTerminateError("manual terminate")
			}

			allDeltas[index] = deform.DeformBone(model, motion, vmd.InitialMotion, true, iFrame, target_bone_names)

			if incrementCompletedCount != nil {
				incrementCompletedCount()
			}

			return nil
		},
		func(iterIndex, allCount int) {
			if logKey != "" {
				processLog(logKey, sizingSet.Index, iterIndex, allCount)
			}
		})
	return allDeltas, err
}

type debugTarget int

const (
	debugTargetOriginal debugTarget = iota
	debugTargetSizing
)

type debugType int

const (
	debugTypeInitial debugType = iota
	debugTypeIdeal
	debugTypeResult
)

// newDebugData デバッグデータを初期化する
func newDebugData(
	allFrames []int, debugBoneNames []pmx.StandardBoneName,
) (positions [][]map[string][]*mmath.MVec3, rotations [][]map[string][]*mmath.MQuaternion) {

	for i := range 2 {
		positions = append(positions, make([]map[string][]*mmath.MVec3, 3))
		rotations = append(rotations, make([]map[string][]*mmath.MQuaternion, 3))
		for j := range 3 {
			positions[i][j] = make(map[string][]*mmath.MVec3)
			rotations[i][j] = make(map[string][]*mmath.MQuaternion)

			for _, debugBoneName := range debugBoneNames {
				for _, direction := range directions {
					boneName := debugBoneName.StringFromDirection(direction)

					positions[i][j][boneName] = make([]*mmath.MVec3, len(allFrames))
					rotations[i][j][boneName] = make([]*mmath.MQuaternion, len(allFrames))
				}
			}
		}
	}

	return positions, rotations
}

// recordDebugData デバッグデータを記録する
func recordDebugData(
	index int, debugBoneNames []pmx.StandardBoneName,
	vmdDeltas *delta.VmdDeltas, debugTarget debugTarget, debugType debugType,
	positions [][]map[string][]*mmath.MVec3, rotations [][]map[string][]*mmath.MQuaternion,
) {
	for _, debugBoneName := range debugBoneNames {
		for _, direction := range directions {
			boneName := debugBoneName.StringFromDirection(direction)
			boneDelta := vmdDeltas.Bones.GetByName(boneName)
			if boneDelta == nil {
				continue
			}
			positions[debugTarget][debugType][boneName][index] = boneDelta.FilledGlobalPosition().Copy()
			rotations[debugTarget][debugType][boneName][index] = boneDelta.FilledFrameRotation().Copy()
		}
	}
}

// outputDebugData デバッグデータの出力
func outputDebugData(
	allFrames []int, debugBoneNames []pmx.StandardBoneName, motionKey, outputPath string, model *pmx.PmxModel,
	positions [][]map[string][]*mmath.MVec3, rotations [][]map[string][]*mmath.MQuaternion,
) {
	motion := vmd.NewVmdMotion("")
	for debugTargetIndex, debugTargetName := range []string{"元", "先"} {
		for debugTypeIndex, debugTypeName := range []string{"今", "理", "結"} {
			for _, debugBoneName := range debugBoneNames {
				for _, direction := range directions {
					boneName := debugBoneName.StringFromDirection(direction)
					config := pmx.BoneConfigFromName(boneName)
					// 出力用ボーン名
					outputBoneName := fmt.Sprintf("%s%s%s%s", debugTargetName, debugTypeName, config.Abbreviation.StringFromDirection(direction), motionKey[len(motionKey)-1:])
					// 出力用ボーン名が存在しない場合はスキップ
					if !model.Bones.ContainsByName(outputBoneName) {
						// mlog.D("[%s] 出力スキップ: %s", motionKey, outputBoneName)
						continue
					}

					for _, iFrame := range allFrames {
						frame := float32(iFrame)
						bf := vmd.NewBoneFrame(frame)
						bf.Position = positions[debugTargetIndex][debugTypeIndex][boneName][iFrame]
						bf.Rotation = rotations[debugTargetIndex][debugTypeIndex][boneName][iFrame]
						if bf.Position == nil && bf.Rotation == nil {
							continue
						}
						motion.InsertBoneFrame(outputBoneName, bf)
					}
				}
			}
		}
	}

	outputVerboseMotion(motionKey, outputPath, motion)
}

func checkBones(
	sizingSet *domain.SizingSet,
	originalTrunkChecks []domain.CheckTrunkBoneType,
	originalDirectionChecks []domain.CheckDirectionBoneType,
	sizingTrunkChecks []domain.CheckTrunkBoneType,
	sizingDirectionChecks []domain.CheckDirectionBoneType,
) error {
	var err error
	errorMessage := make([]string, 0)

	for _, v := range originalTrunkChecks {
		if v.CheckFunk() == nil {
			keyName := "ボーン不足エラー"
			if !v.IsStandard() {
				keyName = "検証ボーン不足エラー"
			}
			message := mi18n.T(keyName, map[string]any{
				"Process": mi18n.T("足補正"), "No": sizingSet.Index + 1,
				"ModelType": "元モデル", "BoneName": v.BoneName.String()})
			mlog.WT(mi18n.T("ボーン不足"), message)
			errorMessage = append(errorMessage, message)
			err = merr.NewNameNotFoundError(v.BoneName.String(), message)
		}
	}

	for _, v := range originalDirectionChecks {
		for _, direction := range directions {
			if v.CheckFunk(direction) == nil {
				keyName := "ボーン不足エラー"
				if !v.IsStandard(direction) {
					keyName = "検証ボーン不足エラー"
				}
				message := mi18n.T(keyName, map[string]any{
					"Process": mi18n.T("足補正"), "No": sizingSet.Index + 1,
					"ModelType": "元モデル", "BoneName": v.BoneName.StringFromDirection(direction)})
				mlog.WT(mi18n.T("ボーン不足"), message)
				errorMessage = append(errorMessage, message)
				err = merr.NewNameNotFoundError(v.BoneName.StringFromDirection(direction), message)
			}
		}
	}

	for _, v := range sizingTrunkChecks {
		if v.CheckFunk() == nil {
			keyName := "ボーン不足エラー"
			if !v.IsStandard() {
				keyName = "検証ボーン不足エラー"
			}
			message := mi18n.T(keyName, map[string]any{
				"Process": mi18n.T("足補正"), "No": sizingSet.Index + 1,
				"ModelType": "先モデル", "BoneName": v.BoneName.String()})
			mlog.WT(mi18n.T("ボーン不足"), message)
			errorMessage = append(errorMessage, message)
			err = merr.NewNameNotFoundError(v.BoneName.String(), message)
		}
	}

	for _, v := range sizingDirectionChecks {
		for _, direction := range directions {
			if v.CheckFunk(direction) == nil {
				keyName := "ボーン不足エラー"
				if !v.IsStandard(direction) {
					keyName = "検証ボーン不足エラー"
				}
				message := mi18n.T(keyName, map[string]any{
					"Process": mi18n.T("足補正"), "No": sizingSet.Index + 1,
					"ModelType": "先モデル", "BoneName": v.BoneName.StringFromDirection(direction)})
				mlog.WT(mi18n.T("ボーン不足"), message)
				errorMessage = append(errorMessage, message)
				err = merr.NewNameNotFoundError(v.BoneName.StringFromDirection(direction), message)
			}
		}
	}

	return err
}

type ISizingUsecase interface {
	Exec(sizingSet *domain.SizingSet, sizingSetCount int, incrementCompletedCount func()) (bool, error)
}

// ログはCPUのサイズに応じて可変でブロッキングして出力する
var log_block_size = runtime.NumCPU() * 50

// 方向
var directions = []pmx.BoneDirection{pmx.BONE_DIRECTION_LEFT, pmx.BONE_DIRECTION_RIGHT}

// 体幹下部ボーン名
var trunk_lower_bone_names = []string{
	pmx.ROOT.String(), pmx.TRUNK_ROOT.String(), pmx.CENTER.String(), pmx.GROOVE.String(), pmx.WAIST.String(),
	pmx.LOWER_ROOT.String(), pmx.LOWER.String(), pmx.LEG_CENTER.String(), pmx.LEG_ROOT.Left(), pmx.LEG_ROOT.Right(),
	pmx.HIP.Left(), pmx.HIP.Right(), pmx.LEG.Left(), pmx.LEG.Right()}

// 足関連ボーン名（左右別）
var leg_direction_bone_names = [][]string{
	{pmx.HIP.Left(), pmx.LEG.Left(), pmx.KNEE.Left(), pmx.HEEL.Left(), pmx.ANKLE.Left(), pmx.ANKLE_GROUND.Left(),
		pmx.TOE_T.Left(), pmx.TOE_P.Left(), pmx.TOE_C.Left(), pmx.LEG_D.Left(), pmx.KNEE_D.Left(),
		pmx.HEEL_D.Left(), pmx.ANKLE_D.Left(), pmx.ANKLE_D_GROUND.Left(), pmx.TOE_T_D.Left(), pmx.TOE_P_D.Left(),
		pmx.TOE_C_D.Left(), pmx.TOE_EX.Left(), pmx.LEG_IK_PARENT.Left(), pmx.LEG_IK.Left(), pmx.TOE_IK.Left()},
	{pmx.HIP.Right(), pmx.LEG.Right(), pmx.KNEE.Right(), pmx.HEEL.Right(), pmx.ANKLE.Right(), pmx.ANKLE_GROUND.Right(),
		pmx.TOE_T.Right(), pmx.TOE_P.Right(), pmx.TOE_C.Right(), pmx.LEG_D.Right(), pmx.KNEE_D.Right(),
		pmx.HEEL_D.Right(), pmx.ANKLE_D.Right(), pmx.ANKLE_D_GROUND.Right(), pmx.TOE_T_D.Right(), pmx.TOE_P_D.Right(),
		pmx.TOE_C_D.Right(), pmx.TOE_EX.Right(), pmx.LEG_IK_PARENT.Right(), pmx.LEG_IK.Right(), pmx.TOE_IK.Right()},
}

// 足関連ボーン名（両方向）
var leg_all_direction_bone_names = append(leg_direction_bone_names[0], leg_direction_bone_names[1]...)

// 全ての下半身ボーン名
var all_lower_leg_bone_names = append(trunk_lower_bone_names, leg_all_direction_bone_names...)

// 上半身系ボーン名
var trunk_upper_bone_names = []string{
	pmx.ROOT.String(), pmx.TRUNK_ROOT.String(), pmx.CENTER.String(), pmx.GROOVE.String(), pmx.WAIST.String(),
	pmx.UPPER_ROOT.String(), pmx.UPPER.String(), pmx.UPPER2.String(), pmx.NECK_ROOT.String(),
	pmx.SHOULDER.Left(), pmx.SHOULDER.Right(), pmx.ARM.Left(), pmx.ARM.Right(), pmx.NECK.String()}

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
