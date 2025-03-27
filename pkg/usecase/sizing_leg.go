package usecase

import (
	"miu200521358/vmd_sizing_t4/pkg/domain"

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

// SizingLeg は、足補正処理を行います。
// 処理内容を機能ごとに分割することで、可読性と保守性を向上させています。
func SizingLeg(
	sizingSet *domain.SizingSet, scale *mmath.MVec3, sizingSetCount, totalProcessCount int, getCompletedCount func() int,
) (bool, error) {
	// 対象外の場合は何もせず終了
	if !sizingSet.IsSizingLeg || sizingSet.CompletedSizingLeg {
		return false, nil
	}
	if !isValidSizingLeg(sizingSet) {
		return false, nil
	}

	originalModel := sizingSet.OriginalModel
	originalMotion := sizingSet.OriginalMotion
	sizingModel := sizingSet.OutputModel
	sizingProcessMotion := sizingSet.OutputMotion.Copy()

	// 初期重心計算用の空 VMD モーション
	initialMotion := vmd.NewVmdMotion("")

	// 元モデルと先モデルの初期重心を計算
	originalInitialGravityPos := computeInitialGravity(originalModel, initialMotion)
	sizingInitialGravityPos := computeInitialGravity(sizingModel, initialMotion)
	gravityRatio := sizingInitialGravityPos.Y / originalInitialGravityPos.Y

	// 元モデルの足対象ボーン（IK対象）取得
	originalLeftAnkleBone, err := originalModel.Bones.GetIkTarget(pmx.LEG_IK.Left())
	if err != nil {
		return false, err
	}
	originalRightAnkleBone, err := originalModel.Bones.GetIkTarget(pmx.LEG_IK.Right())
	if err != nil {
		return false, err
	}

	// サイジング先モデルの各種必要ボーン取得
	sizingCenterBone, _ := sizingModel.Bones.GetCenter()
	sizingGrooveBone, _ := sizingModel.Bones.GetGroove()

	sizingLeftLegIkBone, _ := sizingModel.Bones.GetLegIk(pmx.BONE_DIRECTION_LEFT)
	sizingLeftLegBone, _ := sizingModel.Bones.GetLeg(pmx.BONE_DIRECTION_LEFT)
	sizingLeftKneeBone, _ := sizingModel.Bones.GetKnee(pmx.BONE_DIRECTION_LEFT)
	sizingLeftAnkleBone, _ := sizingModel.Bones.GetIkTarget(pmx.LEG_IK.Left())
	sizingLeftToeIkBone, _ := sizingModel.Bones.GetToeIK(pmx.BONE_DIRECTION_LEFT)
	sizingLeftToeBone, _ := sizingModel.Bones.GetIkTarget(pmx.TOE_IK.Left())

	sizingRightLegIkBone, _ := sizingModel.Bones.GetLegIk(pmx.BONE_DIRECTION_RIGHT)
	sizingRightLegBone, _ := sizingModel.Bones.GetLeg(pmx.BONE_DIRECTION_RIGHT)
	sizingRightKneeBone, _ := sizingModel.Bones.GetKnee(pmx.BONE_DIRECTION_RIGHT)
	sizingRightAnkleBone, _ := sizingModel.Bones.GetIkTarget(pmx.LEG_IK.Right())
	sizingRightToeIkBone, _ := sizingModel.Bones.GetToeIK(pmx.BONE_DIRECTION_RIGHT)
	sizingRightToeBone, _ := sizingModel.Bones.GetIkTarget(pmx.TOE_IK.Right())

	frames := sizingProcessMotion.BoneFrames.RegisteredIndexesByNames(all_lower_leg_bone_names)
	if len(frames) == 0 {
		return false, nil
	}
	blockSize, _ := miter.GetBlockSize(len(frames) * sizingSetCount)

	// 元モデルのデフォーム結果を並列処理で取得
	originalAllDeltas, err := computeOriginalDeltas(frames, blockSize, originalModel, originalMotion, originalLeftAnkleBone, originalRightAnkleBone, sizingSet, totalProcessCount, getCompletedCount)
	if err != nil {
		return false, err
	}

	processLog("足補正開始", sizingSet.Index, getCompletedCount(), totalProcessCount, 0, 1)

	// サイジング先モデルに対して FK 焼き込み処理
	if err := bakeFK(sizingProcessMotion, originalAllDeltas, sizingSet); err != nil {
		return false, err
	}

	// 詳細出力（IK フレームの挿入）
	if mlog.IsVerbose() {
		insertVerboseIKFrame(sizingProcessMotion, sizingSet.OriginalMotionPath, sizingLeftLegIkBone, sizingLeftToeIkBone, sizingRightLegIkBone, sizingRightToeIkBone)
	}

	// TOE_IK フレームのリセット
	sizingProcessMotion.BoneFrames.Delete(pmx.TOE_IK.Left())
	sizingProcessMotion.BoneFrames.Delete(pmx.TOE_IK.Right())
	sizingProcessMotion.BoneFrames.Append(vmd.NewBoneNameFrames(pmx.TOE_IK.Left()))
	sizingProcessMotion.BoneFrames.Append(vmd.NewBoneNameFrames(pmx.TOE_IK.Right()))

	// センター・グルーブ補正を実施
	centerPositions, groovePositions, err := applyCenterCorrection(frames, blockSize, gravityRatio, scale, originalAllDeltas, sizingModel, sizingProcessMotion, sizingCenterBone, sizingGrooveBone, sizingSet, totalProcessCount, getCompletedCount)
	if err != nil {
		return false, err
	}
	updateCenterGroovePositions(frames, sizingProcessMotion, sizingCenterBone, sizingGrooveBone, centerPositions, groovePositions)

	if mlog.IsVerbose() {
		outputMotion("足補正07_センター補正", sizingSet.OriginalMotionPath, sizingProcessMotion)
	}

	// 足IK 補正処理
	leftLegIkPositions, leftLegIkRotations, rightLegIkPositions, rightLegIkRotations, err := processLegIKCorrection(
		frames, blockSize, sizingModel, sizingProcessMotion, originalAllDeltas,
		originalLeftAnkleBone, originalRightAnkleBone,
		sizingLeftLegIkBone, sizingLeftToeBone, sizingLeftToeIkBone,
		sizingRightLegIkBone, sizingRightToeBone, sizingRightToeIkBone,
		scale, sizingSet, totalProcessCount, getCompletedCount,
	)
	if err != nil {
		return false, err
	}
	updateLegIKFrames(frames, sizingProcessMotion, sizingLeftLegIkBone, sizingRightLegIkBone, leftLegIkPositions, leftLegIkRotations, rightLegIkPositions, rightLegIkRotations, originalAllDeltas)

	if mlog.IsVerbose() {
		outputMotion("足補正08_足IK補正", sizingSet.OriginalMotionPath, sizingProcessMotion)
	}

	// 足FK 再計算（IK ON状態）
	leftLegRotations, leftKneeRotations, leftAnkleRotations,
		rightLegRotations, rightKneeRotations, rightAnkleRotations, err :=
		recalculateLegFKRotations(frames, blockSize, sizingModel, sizingProcessMotion,
			sizingLeftLegBone, sizingLeftKneeBone, sizingLeftAnkleBone,
			sizingRightLegBone, sizingRightKneeBone, sizingRightAnkleBone,
			sizingSet, totalProcessCount, getCompletedCount)
	if err != nil {
		return false, err
	}

	// 再計算した回転情報をサイジング先モーションに反映
	registerLegFk(frames, sizingProcessMotion,
		sizingLeftLegBone, sizingLeftKneeBone, sizingLeftAnkleBone,
		sizingRightLegBone, sizingRightKneeBone, sizingRightAnkleBone,
		leftLegRotations, leftKneeRotations, leftAnkleRotations,
		rightLegRotations, rightKneeRotations, rightAnkleRotations,
		sizingSet)

	if mlog.IsVerbose() {
		outputMotion("足補正09_FK再計算", sizingSet.OriginalMotionPath, sizingProcessMotion)
	}

	sizingSet.OutputMotion = sizingProcessMotion
	sizingSet.CompletedSizingLeg = true

	return true, nil
}
func outputMotion(title string, originalMotionPath string, motion *vmd.VmdMotion) {
	outputPath := mfile.CreateOutputPath(originalMotionPath, title)
	repository.NewVmdRepository().Save(outputPath, motion, true)
	mlog.V("%s: %s", title, outputPath)
}

// computeInitialGravity は、対象モデルの初期重心位置を計算します。
func computeInitialGravity(model *pmx.PmxModel, initialMotion *vmd.VmdMotion) *mmath.MVec3 {
	vmdDeltas := delta.NewVmdDeltas(0, model.Bones, model.Hash(), initialMotion.Hash())
	vmdDeltas.Morphs = deform.DeformMorph(model, initialMotion.MorphFrames, 0, nil)
	vmdDeltas.Bones = deform.DeformBone(model, initialMotion, true, 0, gravity_bone_names)
	return calcGravity(vmdDeltas)
}

// computeOriginalDeltas は、元モデルの各フレームごとのデフォーム結果を並列処理で取得します。
func computeOriginalDeltas(
	frames []int, blockSize int,
	originalModel *pmx.PmxModel, originalMotion *vmd.VmdMotion,
	originalLeftAnkleBone, originalRightAnkleBone *pmx.Bone,
	sizingSet *domain.SizingSet, totalProcessCount int, getCompletedCount func() int,
) ([]*delta.VmdDeltas, error) {
	originalAllDeltas := make([]*delta.VmdDeltas, len(frames))
	err := miter.IterParallelByList(frames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}
			frame := float32(data)
			vmdDeltas := delta.NewVmdDeltas(frame, originalModel.Bones, originalModel.Hash(), originalMotion.Hash())
			vmdDeltas.Morphs = deform.DeformMorph(originalModel, originalMotion.MorphFrames, frame, nil)
			// 足補正で必要なボーン群（重心および下半身・足）を対象とする
			vmdDeltas.Bones = deform.DeformBone(originalModel, originalMotion, true, data, append(all_gravity_lower_leg_bone_names, originalLeftAnkleBone.Name(), originalRightAnkleBone.Name()))
			originalAllDeltas[index] = vmdDeltas
			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正01", sizingSet.Index, getCompletedCount(), totalProcessCount, iterIndex, allCount)
		})
	return originalAllDeltas, err
}

// bakeFK は、元モデルのデフォーム結果から FK 回転をサイジング先モーションに焼き込みます。
func bakeFK(sizingProcessMotion *vmd.VmdMotion, originalAllDeltas []*delta.VmdDeltas, sizingSet *domain.SizingSet) error {
	for _, vmdDeltas := range originalAllDeltas {
		if sizingSet.IsTerminate {
			return merr.TerminateError
		}
		// 足（LEG）の回転補正
		for _, boneName := range []string{pmx.LEG.Left(), pmx.LEG.Right()} {
			boneDelta := vmdDeltas.Bones.GetByName(boneName)
			if boneDelta == nil {
				continue
			}
			lowerDelta := vmdDeltas.Bones.GetByName(pmx.LOWER.String())
			sizingBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(boneDelta.Frame)
			sizingBf.Rotation = lowerDelta.FilledGlobalMatrix().Inverted().Muled(boneDelta.FilledGlobalMatrix()).Quaternion()
			sizingProcessMotion.InsertRegisteredBoneFrame(boneName, sizingBf)
		}
		// ひざ・足首の回転補正
		for _, boneName := range []string{pmx.KNEE.Left(), pmx.KNEE.Right(), pmx.ANKLE.Left(), pmx.ANKLE.Right()} {
			boneDelta := vmdDeltas.Bones.GetByName(boneName)
			if boneDelta == nil {
				continue
			}
			sizingBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(boneDelta.Frame)
			sizingBf.Rotation = boneDelta.FilledFrameRotation()
			sizingProcessMotion.InsertRegisteredBoneFrame(boneName, sizingBf)
		}
	}
	return nil
}

// newIkEnableFrameWithBone は、与えられたボーン名と有効フラグから新しい IK 有効フレームを生成するヘルパー関数です。
func newIkEnableFrameWithBone(boneName string, enabled bool) *vmd.IkEnabledFrame {
	// 0 はフレーム番号の初期値として設定（必要に応じて変更してください）
	frame := vmd.NewIkEnableFrame(0)
	frame.BoneName = boneName
	frame.Enabled = enabled
	return frame
}

// insertVerboseIKFrame は、verbose モード時に IK フレームを挿入して中間結果を出力します。
func insertVerboseIKFrame(sizingProcessMotion *vmd.VmdMotion, motionPath string,
	sizingLeftLegIkBone, sizingLeftToeIkBone, sizingRightLegIkBone, sizingRightToeIkBone *pmx.Bone,
) {
	kf := vmd.NewIkFrame(0)
	kf.Registered = true
	// 左足
	kf.IkList = append(kf.IkList, newIkEnableFrameWithBone(sizingLeftLegIkBone.Name(), false))
	kf.IkList = append(kf.IkList, newIkEnableFrameWithBone(sizingLeftToeIkBone.Name(), false))
	// 右足
	kf.IkList = append(kf.IkList, newIkEnableFrameWithBone(sizingRightLegIkBone.Name(), false))
	kf.IkList = append(kf.IkList, newIkEnableFrameWithBone(sizingRightToeIkBone.Name(), false))
	sizingProcessMotion.InsertIkFrame(kf)

	outputPath := mfile.CreateOutputPath(motionPath, "足補正01_FK焼き込み")
	repository.NewVmdRepository().Save(outputPath, sizingProcessMotion, true)
	mlog.V("足補正01_FK焼き込み: %s", outputPath)
}

// applyCenterCorrection は、センターおよびグルーブの位置補正を並列処理で計算します。
func applyCenterCorrection(
	frames []int, blockSize int, gravityRatio float64, scale *mmath.MVec3,
	originalAllDeltas []*delta.VmdDeltas, sizingModel *pmx.PmxModel, sizingProcessMotion *vmd.VmdMotion,
	sizingCenterBone, sizingGrooveBone *pmx.Bone, sizingSet *domain.SizingSet,
	totalProcessCount int, getCompletedCount func() int,
) ([]*mmath.MVec3, []*mmath.MVec3, error) {
	centerPositions := make([]*mmath.MVec3, len(frames))
	groovePositions := make([]*mmath.MVec3, len(frames))
	var gravityMotion *vmd.VmdMotion
	if mlog.IsVerbose() {
		gravityMotion = vmd.NewVmdMotion("")
	}
	err := miter.IterParallelByList(frames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}
			frame := float32(data)
			vmdDeltas := delta.NewVmdDeltas(frame, sizingModel.Bones, sizingModel.Hash(), sizingProcessMotion.Hash())
			vmdDeltas.Morphs = deform.DeformMorph(sizingModel, sizingProcessMotion.MorphFrames, frame, nil)
			vmdDeltas.Bones = deform.DeformBone(sizingModel, sizingProcessMotion, false, data, gravity_bone_names)

			originalGravityPos := calcGravity(originalAllDeltas[index])
			sizingGravityPos := calcGravity(vmdDeltas)
			sizingFixCenterTargetY := originalGravityPos.Y * gravityRatio
			yDiff := sizingFixCenterTargetY - sizingGravityPos.Y

			mlog.V("足補正07[%04.0f] originalY[%.4f], sizingY[%.4f], sizingFixY[%.4f], diff[%.4f]",
				frame, originalGravityPos.Y, sizingGravityPos.Y, sizingFixCenterTargetY, yDiff)

			if mlog.IsVerbose() && gravityMotion != nil {
				bf := vmd.NewBoneFrame(frame)
				bf.Position = originalGravityPos
				gravityMotion.InsertRegisteredBoneFrame("元重心", bf)
				bf = vmd.NewBoneFrame(frame)
				bf.Position = sizingGravityPos
				gravityMotion.InsertRegisteredBoneFrame("先重心", bf)
			}

			sizingCenterBf := sizingProcessMotion.BoneFrames.Get(sizingCenterBone.Name()).Get(frame)
			centerPositions[index] = sizingCenterBf.Position.Muled(scale)

			sizingGrooveBf := sizingProcessMotion.BoneFrames.Get(sizingGrooveBone.Name()).Get(frame)
			groovePositions[index] = sizingGrooveBf.Position.Added(&mmath.MVec3{X: 0, Y: yDiff, Z: 0})

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正07", sizingSet.Index, getCompletedCount(), totalProcessCount, iterIndex, allCount)
		})
	// verbose時は中間結果の出力も実施
	if mlog.IsVerbose() && gravityMotion != nil {
		path := mfile.CreateOutputPath(sizingSet.OriginalMotionPath, "gravity")
		repository.NewVmdRepository().Save(path, gravityMotion, true)
	}
	return centerPositions, groovePositions, err
}

// updateCenterGroovePositions は、計算したセンター・グルーブ位置をサイジング先モーションに反映します。
func updateCenterGroovePositions(
	frames []int, sizingProcessMotion *vmd.VmdMotion,
	sizingCenterBone, sizingGrooveBone *pmx.Bone,
	centerPositions, groovePositions []*mmath.MVec3,
) {
	for i, iFrame := range frames {
		frame := float32(iFrame)
		sizingCenterBf := sizingProcessMotion.BoneFrames.Get(sizingCenterBone.Name()).Get(frame)
		sizingCenterBf.Position = centerPositions[i]
		sizingProcessMotion.InsertRegisteredBoneFrame(sizingCenterBone.Name(), sizingCenterBf)

		sizingGrooveBf := sizingProcessMotion.BoneFrames.Get(sizingGrooveBone.Name()).Get(frame)
		sizingGrooveBf.Position = groovePositions[i]
		sizingProcessMotion.InsertRegisteredBoneFrame(sizingGrooveBone.Name(), sizingGrooveBf)
	}
}

// processLegIKCorrection は、足IK 補正の計算を並列処理で行い、各フレームごとの位置・回転補正値を算出します。
func processLegIKCorrection(
	frames []int, blockSize int, sizingModel *pmx.PmxModel, sizingProcessMotion *vmd.VmdMotion,
	originalAllDeltas []*delta.VmdDeltas, originalLeftAnkleBone, originalRightAnkleBone *pmx.Bone,
	sizingLeftLegIkBone, sizingLeftToeBone, sizingLeftToeIkBone,
	sizingRightLegIkBone, sizingRightToeBone, sizingRightToeIkBone *pmx.Bone,
	scale *mmath.MVec3, sizingSet *domain.SizingSet, totalProcessCount int, getCompletedCount func() int,
) ([]*mmath.MVec3, []*mmath.MQuaternion, []*mmath.MVec3, []*mmath.MQuaternion, error) {
	leftLegIkPositions := make([]*mmath.MVec3, len(frames))
	leftLegIkRotations := make([]*mmath.MQuaternion, len(frames))
	rightLegIkPositions := make([]*mmath.MVec3, len(frames))
	rightLegIkRotations := make([]*mmath.MQuaternion, len(frames))
	err := miter.IterParallelByList(frames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}
			frame := float32(data)
			vmdDeltas := delta.NewVmdDeltas(frame, sizingModel.Bones, sizingModel.Hash(), sizingProcessMotion.Hash())
			vmdDeltas.Morphs = deform.DeformMorph(sizingModel, sizingProcessMotion.MorphFrames, frame, nil)
			vmdDeltas.Bones = deform.DeformBone(sizingModel, sizingProcessMotion, false, data, all_lower_leg_bone_names)

			// 元モデルから各種足ボーンの位置取得
			originalLeftAnklePosition := originalAllDeltas[index].Bones.GetByName(pmx.ANKLE.Left()).FilledGlobalPosition()
			originalRightAnklePosition := originalAllDeltas[index].Bones.GetByName(pmx.ANKLE.Right()).FilledGlobalPosition()
			originalLeftHeelDelta := originalAllDeltas[index].Bones.GetByName(pmx.HEEL.Left())
			originalLeftToeTailDelta := originalAllDeltas[index].Bones.GetByName(pmx.TOE_T.Left())
			originalRightHeelDelta := originalAllDeltas[index].Bones.GetByName(pmx.HEEL.Right())
			originalRightToeTailDelta := originalAllDeltas[index].Bones.GetByName(pmx.TOE_T.Right())

			// サイジング先モデルの各足ボーンのデフォーム結果取得
			sizingLeftAnkleDelta := vmdDeltas.Bones.GetByName(pmx.ANKLE.Left())
			sizingLeftHeelDelta := vmdDeltas.Bones.GetByName(pmx.HEEL.Left())
			sizingLeftToeDelta := vmdDeltas.Bones.GetByName(sizingLeftToeBone.Name())
			sizingLeftToeTailDelta := vmdDeltas.Bones.GetByName(pmx.TOE_T.Left())

			sizingRightAnkleDelta := vmdDeltas.Bones.GetByName(pmx.ANKLE.Right())
			sizingRightHeelDelta := vmdDeltas.Bones.GetByName(pmx.HEEL.Right())
			sizingRightToeDelta := vmdDeltas.Bones.GetByName(sizingRightToeBone.Name())
			sizingRightToeTailDelta := vmdDeltas.Bones.GetByName(pmx.TOE_T.Right())

			// 足IK の初期位置計算（足首位置から各IK ボーンへのオフセット）
			leftLegIkPositions[index] = sizingLeftAnkleDelta.FilledGlobalPosition().Subed(sizingLeftLegIkBone.Position)
			rightLegIkPositions[index] = sizingRightAnkleDelta.FilledGlobalPosition().Subed(sizingRightLegIkBone.Position)

			// 足IK の Y 軸補正
			calcLegIkPositionY(index, frame, pmx.BONE_DIRECTION_LEFT, leftLegIkPositions,
				originalLeftAnkleBone, originalLeftAnklePosition,
				originalLeftToeTailDelta, originalLeftHeelDelta,
				sizingLeftToeTailDelta, sizingLeftHeelDelta, scale)
			calcLegIkPositionY(index, frame, pmx.BONE_DIRECTION_RIGHT, rightLegIkPositions,
				originalRightAnkleBone, originalRightAnklePosition,
				originalRightToeTailDelta, originalRightHeelDelta,
				sizingRightToeTailDelta, sizingRightHeelDelta, scale)

			// 足IK の回転補正（つま先 IK から算出）
			leftLegIkMat := sizingLeftToeIkBone.Position.Subed(sizingLeftLegIkBone.Position).Normalize().ToLocalMat()
			leftLegFkMat := sizingLeftToeDelta.FilledGlobalPosition().Subed(sizingLeftAnkleDelta.FilledGlobalPosition()).Normalize().ToLocalMat()
			leftLegIkRotations[index] = leftLegFkMat.Muled(leftLegIkMat.Inverted()).Quaternion()

			rightLegIkMat := sizingRightToeIkBone.Position.Subed(sizingRightLegIkBone.Position).Normalize().ToLocalMat()
			rightLegFkMat := sizingRightToeDelta.FilledGlobalPosition().Subed(sizingRightAnkleDelta.FilledGlobalPosition()).Normalize().ToLocalMat()
			rightLegIkRotations[index] = rightLegFkMat.Muled(rightLegIkMat.Inverted()).Quaternion()

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正08", sizingSet.Index, getCompletedCount(), totalProcessCount, iterIndex, allCount)
		})
	return leftLegIkPositions, leftLegIkRotations, rightLegIkPositions, rightLegIkRotations, err
}

// updateLegIKFrames は、計算済みの足IK 補正位置と回転値をモーションデータに反映させます。
func updateLegIKFrames(
	frames []int, sizingProcessMotion *vmd.VmdMotion,
	sizingLeftLegIkBone, sizingRightLegIkBone *pmx.Bone,
	leftLegIkPositions []*mmath.MVec3, leftLegIkRotations []*mmath.MQuaternion,
	rightLegIkPositions []*mmath.MVec3, rightLegIkRotations []*mmath.MQuaternion,
	originalAllDeltas []*delta.VmdDeltas,
) {
	for i, iFrame := range frames {
		if i > 0 {
			// 前フレームと同じ足首位置なら前フレームの値を継承
			originalLeftAnklePosition := originalAllDeltas[i].Bones.GetByName(pmx.ANKLE.Left()).FilledGlobalPosition()
			originalRightAnklePosition := originalAllDeltas[i].Bones.GetByName(pmx.ANKLE.Right()).FilledGlobalPosition()
			originalLeftAnklePrevPosition := originalAllDeltas[i-1].Bones.GetByName(pmx.ANKLE.Left()).FilledGlobalPosition()
			originalRightAnklePrevPosition := originalAllDeltas[i-1].Bones.GetByName(pmx.ANKLE.Right()).FilledGlobalPosition()
			if mmath.NearEquals(originalLeftAnklePrevPosition.X, originalLeftAnklePosition.X, 1e-2) {
				leftLegIkPositions[i].X = leftLegIkPositions[i-1].X
			}
			if mmath.NearEquals(originalLeftAnklePrevPosition.Y, originalLeftAnklePosition.Y, 1e-2) {
				leftLegIkPositions[i].Y = leftLegIkPositions[i-1].Y
			}
			if mmath.NearEquals(originalLeftAnklePrevPosition.Z, originalLeftAnklePosition.Z, 1e-2) {
				leftLegIkPositions[i].Z = leftLegIkPositions[i-1].Z
			}
			if mmath.NearEquals(originalRightAnklePrevPosition.X, originalRightAnklePosition.X, 1e-2) {
				rightLegIkPositions[i].X = rightLegIkPositions[i-1].X
			}
			if mmath.NearEquals(originalRightAnklePrevPosition.Y, originalRightAnklePosition.Y, 1e-2) {
				rightLegIkPositions[i].Y = rightLegIkPositions[i-1].Y
			}
			if mmath.NearEquals(originalRightAnklePrevPosition.Z, originalRightAnklePosition.Z, 1e-2) {
				rightLegIkPositions[i].Z = rightLegIkPositions[i-1].Z
			}
		}
		frame := float32(iFrame)
		// 各フレームの IK 補正値を更新
		rightLegIkBf := sizingProcessMotion.BoneFrames.Get(sizingRightLegIkBone.Name()).Get(frame)
		rightLegIkBf.Position = rightLegIkPositions[i]
		rightLegIkBf.Rotation = rightLegIkRotations[i]
		sizingProcessMotion.InsertRegisteredBoneFrame(sizingRightLegIkBone.Name(), rightLegIkBf)

		leftLegIkBf := sizingProcessMotion.BoneFrames.Get(sizingLeftLegIkBone.Name()).Get(frame)
		leftLegIkBf.Position = leftLegIkPositions[i]
		leftLegIkBf.Rotation = leftLegIkRotations[i]
		sizingProcessMotion.InsertRegisteredBoneFrame(sizingLeftLegIkBone.Name(), leftLegIkBf)
	}
}

// recalculateLegFKRotations は、IK ON状態で足FK の回転を再計算し、その結果を配列で返します。
func recalculateLegFKRotations(
	frames []int, blockSize int, sizingModel *pmx.PmxModel, sizingProcessMotion *vmd.VmdMotion,
	sizingLeftLegBone, sizingLeftKneeBone, sizingLeftAnkleBone,
	sizingRightLegBone, sizingRightKneeBone, sizingRightAnkleBone *pmx.Bone,
	sizingSet *domain.SizingSet, totalProcessCount int, getCompletedCount func() int,
) ([]*mmath.MQuaternion, []*mmath.MQuaternion, []*mmath.MQuaternion,
	[]*mmath.MQuaternion, []*mmath.MQuaternion, []*mmath.MQuaternion, error) {
	leftLegRotations := make([]*mmath.MQuaternion, len(frames))
	leftKneeRotations := make([]*mmath.MQuaternion, len(frames))
	leftAnkleRotations := make([]*mmath.MQuaternion, len(frames))
	rightLegRotations := make([]*mmath.MQuaternion, len(frames))
	rightKneeRotations := make([]*mmath.MQuaternion, len(frames))
	rightAnkleRotations := make([]*mmath.MQuaternion, len(frames))
	err := miter.IterParallelByList(frames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}
			frame := float32(data)
			vmdDeltas := delta.NewVmdDeltas(frame, sizingModel.Bones, sizingModel.Hash(), sizingProcessMotion.Hash())
			vmdDeltas.Morphs = deform.DeformMorph(sizingModel, sizingProcessMotion.MorphFrames, frame, nil)
			vmdDeltas.Bones = deform.DeformBone(sizingModel, sizingProcessMotion, true, data, all_lower_leg_bone_names)
			leftLegRotations[index] = vmdDeltas.Bones.Get(sizingLeftLegBone.Index()).FilledFrameRotation()
			leftKneeRotations[index] = vmdDeltas.Bones.Get(sizingLeftKneeBone.Index()).FilledFrameRotation()
			leftAnkleRotations[index] = vmdDeltas.Bones.Get(sizingLeftAnkleBone.Index()).FilledFrameRotation()
			rightLegRotations[index] = vmdDeltas.Bones.Get(sizingRightLegBone.Index()).FilledFrameRotation()
			rightKneeRotations[index] = vmdDeltas.Bones.Get(sizingRightKneeBone.Index()).FilledFrameRotation()
			rightAnkleRotations[index] = vmdDeltas.Bones.Get(sizingRightAnkleBone.Index()).FilledFrameRotation()
			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正09", sizingSet.Index, getCompletedCount(), totalProcessCount, iterIndex, allCount)
		})
	return leftLegRotations, leftKneeRotations, leftAnkleRotations, rightLegRotations, rightKneeRotations, rightAnkleRotations, err
}

func calcLegIkPositionY(
	index int,
	frame float32,
	direction pmx.BoneDirection,
	legIkPositions []*mmath.MVec3,
	originalAnkleBone *pmx.Bone,
	originalAnklePosition *mmath.MVec3,
	originalToeTailDelta, originalHeelDelta, sizingToeTailDelta, sizingHeelDelta *delta.BoneDelta,
	scale *mmath.MVec3,
) {

	// 左足IK-Yの位置を調整
	if mmath.NearEquals(originalAnklePosition.Y, 0, 1e-2) {
		legIkPositions[index].Y = 0
		return
	}

	if originalToeTailDelta.FilledGlobalPosition().Y <= originalHeelDelta.FilledGlobalPosition().Y {
		// つま先の方がかかとより低い場合
		originalLeftToeTailY := originalToeTailDelta.FilledGlobalPosition().Y

		// つま先のY座標を元モデルのつま先のY座標*スケールに合わせる
		sizingLeftToeTailY := originalLeftToeTailY * scale.Y

		// 現時点のつま先のY座標
		actualLeftToeTailY := sizingToeTailDelta.FilledGlobalPosition().Y

		leftToeDiff := sizingLeftToeTailY - actualLeftToeTailY
		lerpLeftToeDiff := mmath.Lerp(leftToeDiff, 0,
			originalToeTailDelta.FilledGlobalPosition().Y/originalAnkleBone.Position.Y)
		// 足首Y位置に近付くにつれて補正を弱める
		legIkPositions[index].Y += lerpLeftToeDiff
		mlog.V("足補正08[%04.0f][%sつま先] originalLeftY[%.4f], sizingLeftY[%.4f], actualLeftY[%.4f], diff[%.4f], lerp[%.4f]",
			frame, direction, originalLeftToeTailY, sizingLeftToeTailY, actualLeftToeTailY, leftToeDiff, lerpLeftToeDiff)

		return
	}

	// かかとの方がつま先より低い場合
	originalLeftHeelY := originalHeelDelta.FilledGlobalPosition().Y

	// かかとのY座標を元モデルのかかとのY座標*スケールに合わせる
	sizingLeftHeelY := originalLeftHeelY * scale.Y

	// 現時点のかかとのY座標
	actualLeftHeelY := sizingHeelDelta.FilledGlobalPosition().Y

	leftHeelDiff := sizingLeftHeelY - actualLeftHeelY
	lerpLeftHeelDiff := mmath.Lerp(leftHeelDiff, 0,
		originalHeelDelta.FilledGlobalPosition().Y/originalAnkleBone.Position.Y)
	// 足首Y位置に近付くにつれて補正を弱める
	legIkPositions[index].Y += lerpLeftHeelDiff

	mlog.V("足補正08[%04.0f][%sかかと] originalLeftY[%.4f], sizingLeftY[%.4f], actualLeftY[%.4f], diff[%.4f], lerp[%.4f]",
		frame, direction, originalLeftHeelY, sizingLeftHeelY, actualLeftHeelY, leftHeelDiff, lerpLeftHeelDiff)
}

func registerLegFk(
	frames []int,
	sizingMotion *vmd.VmdMotion,
	sizingLeftLegBone, sizingLeftKneeBone, sizingLeftAnkleBone,
	sizingRightLegBone, sizingRightKneeBone, sizingRightAnkleBone *pmx.Bone,
	leftLegRotations, leftKneeRotations, leftAnkleRotations,
	rightLegRotations, rightKneeRotations, rightAnkleRotations []*mmath.MQuaternion,
	sizingSet *domain.SizingSet,
) (sizingLeftFkAllDeltas, sizingRightFkAllDeltas []*delta.VmdDeltas) {

	// サイジング先にFKを焼き込み
	for i, iFrame := range frames {
		if sizingSet.IsTerminate {
			return sizingLeftFkAllDeltas, sizingRightFkAllDeltas
		}

		frame := float32(iFrame)

		{
			bf := sizingMotion.BoneFrames.Get(sizingLeftLegBone.Name()).Get(frame)
			bf.Rotation = leftLegRotations[i]
			sizingMotion.InsertRegisteredBoneFrame(sizingLeftLegBone.Name(), bf)
		}
		{
			bf := sizingMotion.BoneFrames.Get(sizingLeftKneeBone.Name()).Get(frame)
			bf.Rotation = leftKneeRotations[i]
			sizingMotion.InsertRegisteredBoneFrame(sizingLeftKneeBone.Name(), bf)
		}
		{
			bf := sizingMotion.BoneFrames.Get(sizingLeftAnkleBone.Name()).Get(frame)
			bf.Rotation = leftAnkleRotations[i]
			sizingMotion.InsertRegisteredBoneFrame(sizingLeftAnkleBone.Name(), bf)
		}
		{
			bf := sizingMotion.BoneFrames.Get(sizingRightLegBone.Name()).Get(frame)
			bf.Rotation = rightLegRotations[i]
			sizingMotion.InsertRegisteredBoneFrame(sizingRightLegBone.Name(), bf)
		}
		{
			bf := sizingMotion.BoneFrames.Get(sizingRightKneeBone.Name()).Get(frame)
			bf.Rotation = rightKneeRotations[i]
			sizingMotion.InsertRegisteredBoneFrame(sizingRightKneeBone.Name(), bf)
		}
		{
			bf := sizingMotion.BoneFrames.Get(sizingRightAnkleBone.Name()).Get(frame)
			bf.Rotation = rightAnkleRotations[i]
			sizingMotion.InsertRegisteredBoneFrame(sizingRightAnkleBone.Name(), bf)
		}
	}

	return sizingLeftFkAllDeltas, sizingRightFkAllDeltas
}

func isValidSizingLeg(sizingSet *domain.SizingSet) bool {
	originalModel := sizingSet.OriginalModel
	sizingModel := sizingSet.OutputModel

	// センター、グルーブ、下半身、右足IK、左足IKが存在するか

	if !originalModel.Bones.ContainsByName(pmx.CENTER.String()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("元モデル"), "BoneName": pmx.CENTER.String()}))
		return false
	}

	if !originalModel.Bones.ContainsByName(pmx.GROOVE.String()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("元モデル"), "BoneName": pmx.GROOVE.String()}))
		return false
	}

	if !originalModel.Bones.ContainsByName(pmx.LOWER.String()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("元モデル"), "BoneName": pmx.LOWER.String()}))
		return false
	}

	// ------------------------------

	if !originalModel.Bones.ContainsByName(pmx.LEG_IK.Left()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("元モデル"), "BoneName": pmx.LEG_IK.Left()}))
		return false
	}

	if !originalModel.Bones.ContainsByName(pmx.LEG.Left()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("元モデル"), "BoneName": pmx.LEG.Left()}))
		return false
	}

	if !originalModel.Bones.ContainsByName(pmx.KNEE.Left()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("元モデル"), "BoneName": pmx.KNEE.Left()}))
		return false
	}

	if !originalModel.Bones.ContainsByName(pmx.ANKLE.Left()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("元モデル"), "BoneName": pmx.ANKLE.Left()}))
		return false
	}

	if !originalModel.Bones.ContainsByName(pmx.TOE_IK.Left()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("元モデル"), "BoneName": pmx.TOE_IK.Left()}))
		return false
	}

	// ------------------------------

	if !originalModel.Bones.ContainsByName(pmx.LEG_IK.Right()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("元モデル"), "BoneName": pmx.LEG_IK.Right()}))
		return false
	}

	if !originalModel.Bones.ContainsByName(pmx.LEG.Right()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("元モデル"), "BoneName": pmx.LEG.Right()}))
		return false
	}

	if !originalModel.Bones.ContainsByName(pmx.KNEE.Right()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("元モデル"), "BoneName": pmx.KNEE.Right()}))
		return false
	}

	if !originalModel.Bones.ContainsByName(pmx.ANKLE.Right()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("元モデル"), "BoneName": pmx.ANKLE.Right()}))
		return false
	}

	if !originalModel.Bones.ContainsByName(pmx.TOE_IK.Right()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("元モデル"), "BoneName": pmx.TOE_IK.Left()}))
		return false
	}

	// ------------------------------

	if !sizingModel.Bones.ContainsByName(pmx.CENTER.String()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("先モデル"), "BoneName": pmx.CENTER.String()}))
		return false
	}

	if !sizingModel.Bones.ContainsByName(pmx.GROOVE.String()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("先モデル"), "BoneName": pmx.GROOVE.String()}))
		return false
	}

	if !sizingModel.Bones.ContainsByName(pmx.LOWER.String()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("先モデル"), "BoneName": pmx.LOWER.String()}))
		return false
	}

	// ------------------------------

	if !sizingModel.Bones.ContainsByName(pmx.LEG_IK.Left()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("先モデル"), "BoneName": pmx.LEG_IK.Left()}))
		return false
	}

	if !sizingModel.Bones.ContainsByName(pmx.LEG.Left()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("先モデル"), "BoneName": pmx.LEG.Left()}))
		return false
	}

	if !sizingModel.Bones.ContainsByName(pmx.KNEE.Left()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("先モデル"), "BoneName": pmx.KNEE.Left()}))
		return false
	}

	if !sizingModel.Bones.ContainsByName(pmx.ANKLE.Left()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("先モデル"), "BoneName": pmx.ANKLE.Left()}))
		return false
	}

	if !sizingModel.Bones.ContainsByName(pmx.TOE_IK.Left()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("先モデル"), "BoneName": pmx.TOE_IK.Left()}))
		return false
	}

	// ------------------------------

	if !sizingModel.Bones.ContainsByName(pmx.LEG_IK.Right()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("先モデル"), "BoneName": pmx.LEG_IK.Right()}))
		return false
	}

	if !sizingModel.Bones.ContainsByName(pmx.LEG.Right()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("先モデル"), "BoneName": pmx.LEG.Right()}))
		return false
	}

	if !sizingModel.Bones.ContainsByName(pmx.KNEE.Right()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("先モデル"), "BoneName": pmx.KNEE.Right()}))
		return false
	}

	if !sizingModel.Bones.ContainsByName(pmx.ANKLE.Right()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("先モデル"), "BoneName": pmx.ANKLE.Right()}))
		return false
	}

	if !sizingModel.Bones.ContainsByName(pmx.TOE_IK.Right()) {
		mlog.WT(mi18n.T("ボーン不足"), mi18n.T("足補正ボーン不足", map[string]interface{}{
			"No": sizingSet.Index + 1, "ModelType": mi18n.T("先モデル"), "BoneName": pmx.TOE_IK.Left()}))
		return false
	}

	return true
}

func calcGravity(vmdDeltas *delta.VmdDeltas) *mmath.MVec3 {
	gravityPos := mmath.NewMVec3()

	for _, boneName := range gravity_bone_names {
		fromBoneDelta := vmdDeltas.Bones.GetByName(boneName)
		if fromBoneDelta == nil || fromBoneDelta.Bone == nil {
			continue
		}
		gravityBoneNames := fromBoneDelta.Bone.Config().CenterOfGravityBoneNames
		if len(gravityBoneNames) == 0 {
			continue
		}
		toBoneName := gravityBoneNames[0].StringFromDirection(fromBoneDelta.Bone.Direction())
		toBoneDelta := vmdDeltas.Bones.GetByName(toBoneName)
		if toBoneDelta == nil || toBoneDelta.Bone == nil {
			continue
		}
		gravity := fromBoneDelta.Bone.Config().CenterOfGravity

		gravityPos.Add(toBoneDelta.FilledGlobalPosition().Added(
			fromBoneDelta.FilledGlobalPosition()).MuledScalar(0.5 * gravity))
	}

	return gravityPos
}
