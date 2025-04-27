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
			// 体幹中心までの長さ比率
			trunkRootScale := sizingTrunkRoot.Position.Y / originalTrunkRoot.Position.Y

			scales[i] = &mmath.MVec3{X: legLengthScale, Y: trunkRootScale, Z: legLengthScale}
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
	incrementCompletedCount func(),
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
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			frame := float32(data)
			vmdDeltas := delta.NewVmdDeltas(frame, model.Bones, model.Hash(), motion.Hash())
			vmdDeltas.Morphs = deform.DeformBoneMorph(model, motion.MorphFrames, frame, nil)
			vmdDeltas.Bones = deform.DeformBone(model, vmd.InitialMotion, false, data, target_bone_names)
			allDeltas[index] = vmdDeltas

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
	pmx.LEG.Left(), pmx.LEG.Right()}

// 足関連ボーン名（左右別）
var leg_direction_bone_names = [][]string{
	{pmx.LEG.Left(), pmx.KNEE.Left(), pmx.HEEL.Left(), pmx.ANKLE.Left(), pmx.ANKLE_GROUND.Left(),
		pmx.TOE_T.Left(), pmx.TOE_P.Left(), pmx.TOE_C.Left(), pmx.LEG_D.Left(), pmx.KNEE_D.Left(),
		pmx.HEEL_D.Left(), pmx.ANKLE_D.Left(), pmx.ANKLE_D_GROUND.Left(), pmx.TOE_T_D.Left(), pmx.TOE_P_D.Left(),
		pmx.TOE_C_D.Left(), pmx.TOE_EX.Left(), pmx.LEG_IK_PARENT.Left(), pmx.LEG_IK.Left(), pmx.TOE_IK.Left()},
	{pmx.LEG.Right(), pmx.KNEE.Right(), pmx.HEEL.Right(), pmx.ANKLE.Right(), pmx.ANKLE_GROUND.Right(),
		pmx.TOE_T.Right(), pmx.TOE_P.Right(), pmx.TOE_C.Right(), pmx.LEG_D.Right(), pmx.KNEE_D.Right(),
		pmx.HEEL_D.Right(), pmx.ANKLE_D.Right(), pmx.ANKLE_D_GROUND.Right(), pmx.TOE_T_D.Right(), pmx.TOE_P_D.Right(),
		pmx.TOE_C_D.Right(), pmx.TOE_EX.Right(), pmx.LEG_IK_PARENT.Right(), pmx.LEG_IK.Right(), pmx.TOE_IK.Right()},
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
