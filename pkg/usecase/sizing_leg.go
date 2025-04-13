package usecase

import (
	"fmt"
	"miu200521358/vmd_sizing_t4/pkg/domain"

	"github.com/miu200521358/mlib_go/pkg/config/merr"
	"github.com/miu200521358/mlib_go/pkg/config/mi18n"
	"github.com/miu200521358/mlib_go/pkg/config/mlog"
	"github.com/miu200521358/mlib_go/pkg/domain/delta"
	"github.com/miu200521358/mlib_go/pkg/domain/mmath"
	"github.com/miu200521358/mlib_go/pkg/domain/pmx"
	"github.com/miu200521358/mlib_go/pkg/domain/vmd"
	"github.com/miu200521358/mlib_go/pkg/infrastructure/miter"
	"github.com/miu200521358/mlib_go/pkg/usecase/deform"
)

// SizingLeg は、足補正処理を行います。
func SizingLeg(
	sizingSet *domain.SizingSet, moveScale *mmath.MVec3, sizingSetCount int, incrementCompletedCount func(),
) (bool, error) {
	// 対象外の場合は何もせず終了
	if !sizingSet.IsSizingLeg || sizingSet.CompletedSizingLeg {
		return false, nil
	}

	originalMotion := sizingSet.OriginalMotion
	sizingProcessMotion, err := sizingSet.OutputMotion.Copy()
	if err != nil {
		return false, err
	}

	mlog.I(mi18n.T("足補正開始", map[string]interface{}{"No": sizingSet.Index + 1}))

	// 処理対象ボーンチェック
	if err := checkBonesForSizingLeg(sizingSet); err != nil {
		return false, err
	}

	allFrames := mmath.IntRanges(int(originalMotion.MaxFrame()) + 1)
	blockSize, _ := miter.GetBlockSize(len(allFrames) * sizingSetCount)

	// 元モデルのデフォーム結果を並列処理で取得
	originalAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.OriginalConfigModel, originalMotion, sizingSet, true, sizingSet.OriginalConfigModel.Bones.GetStandardBoneNames(), "足補正01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	// サイジング先モデルに対して FK 焼き込み処理
	if err := updateLegFK(sizingProcessMotion, originalAllDeltas, sizingSet, "足01"); err != nil {
		return false, err
	}

	incrementCompletedCount()

	// TOE_IK キーフレームのリセット
	sizingProcessMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.TOE_IK.Left()))
	sizingProcessMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.TOE_IK.Right()))

	// 先モデルのIF OFF デフォーム結果を並列処理で取得
	sizingIkOffAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel, sizingProcessMotion, sizingSet, false, sizingSet.SizingConfigModel.Bones.GetStandardBoneNames(), "足補正01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	// 下半身補正を実施
	if err := calculateAdjustedLower(sizingSet, allFrames, blockSize,
		originalAllDeltas, sizingIkOffAllDeltas, sizingProcessMotion, incrementCompletedCount); err != nil {
		return false, err
	}

	incrementCompletedCount()

	// センター・グルーブ補正を実施
	if err := calculateAdjustedCenter(sizingSet, allFrames, blockSize, moveScale,
		originalAllDeltas, sizingIkOffAllDeltas, sizingProcessMotion, incrementCompletedCount); err != nil {
		return false, err
	}

	incrementCompletedCount()

	// 先モデルのデフォーム結果を並列処理で取得
	sizingOffAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel, sizingProcessMotion, sizingSet, false, all_lower_leg_bone_names, "足補正01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	// 足IK 補正処理
	_, _, err = calculateAdjustedLegIK(
		sizingSet, allFrames, blockSize, moveScale, originalAllDeltas, sizingOffAllDeltas, sizingProcessMotion, incrementCompletedCount,
	)
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	// // 足FK 再計算（IK ON状態）
	// if err := calculateAdjustedLegFK(sizingSet, allFrames, blockSize,
	// 	sizingOffAllDeltas, leftLegAnkleIdealPositions, rightLegAnkleIdealPositions,
	// 	sizingProcessMotion, incrementCompletedCount); err != nil {
	// 	return false, err
	// }

	// incrementCompletedCount()

	// sizingSet.OutputMotion = sizingProcessMotion
	// 足補正処理の結果をサイジング先モーションに反映
	if err = updateLegResultMotion(
		sizingSet, allFrames, blockSize, sizingProcessMotion,
		incrementCompletedCount,
	); err != nil {
		return false, err
	}

	sizingSet.CompletedSizingLeg = true

	return true, nil
}

// computeInitialGravity は、対象モデルの初期重心位置を計算します。
func computeInitialGravity(sizingSet *domain.SizingSet, model *pmx.PmxModel, initialMotion *vmd.VmdMotion) *mmath.MVec3 {
	allVmdDeltas, _ := computeVmdDeltas([]int{0}, 1, model, initialMotion, sizingSet, false, gravity_bone_names, "")
	return calcGravity(allVmdDeltas[0])
}

// updateLegFK は、元モデルのデフォーム結果から FK 回転をサイジング先モーションに焼き込みます。
func updateLegFK(
	sizingProcessMotion *vmd.VmdMotion, allDeltas []*delta.VmdDeltas,
	sizingSet *domain.SizingSet, verboseMotionKey string,
) error {
	for i, vmdDeltas := range allDeltas {
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
			bf := sizingProcessMotion.BoneFrames.Get(boneName).Get(boneDelta.Frame)
			bf.Rotation = lowerDelta.FilledGlobalMatrix().Inverted().Muled(boneDelta.FilledGlobalMatrix()).Quaternion()
			sizingProcessMotion.InsertBoneFrame(boneName, bf)
		}
		// ひざ・足首の回転補正
		for _, boneName := range []string{pmx.KNEE.Left(), pmx.KNEE.Right(), pmx.ANKLE.Left(), pmx.ANKLE.Right()} {
			boneDelta := vmdDeltas.Bones.GetByName(boneName)
			if boneDelta == nil {
				continue
			}
			bf := sizingProcessMotion.BoneFrames.Get(boneName).Get(boneDelta.Frame)
			bf.Rotation = boneDelta.FilledFrameRotation()
			sizingProcessMotion.InsertBoneFrame(boneName, bf)
		}

		if i > 0 && i%1000 == 0 {
			processLog("足補正02", sizingSet.Index, i, len(allDeltas))
		}
	}

	if mlog.IsDebug() {
		insertIKFrames(sizingSet, sizingProcessMotion, false)
		outputVerboseMotion(verboseMotionKey, sizingSet.OutputMotionPath, sizingProcessMotion)
	}

	return nil
}

// insertIKFrames は、verbose モード時に IK フレームを挿入して中間結果を出力します。
func insertIKFrames(sizingSet *domain.SizingSet, sizingProcessMotion *vmd.VmdMotion, enabled bool) {
	kf := vmd.NewIkFrame(0)
	// 左足
	kf.IkList = append(kf.IkList, newIkEnableFrameWithBone(sizingSet.SizingLeftLegIkBone().Name(), enabled))
	kf.IkList = append(kf.IkList, newIkEnableFrameWithBone(sizingSet.SizingLeftToeIkBone().Name(), enabled))
	// 右足
	kf.IkList = append(kf.IkList, newIkEnableFrameWithBone(sizingSet.SizingRightLegIkBone().Name(), enabled))
	kf.IkList = append(kf.IkList, newIkEnableFrameWithBone(sizingSet.SizingRightToeIkBone().Name(), enabled))
	sizingProcessMotion.InsertIkFrame(kf)
}

// newIkEnableFrameWithBone は、与えられたボーン名と有効フラグから新しい IK 有効フレームを生成するヘルパー関数です。
func newIkEnableFrameWithBone(boneName string, enabled bool) *vmd.IkEnabledFrame {
	// 0 はフレーム番号の初期値として設定（必要に応じて変更してください）
	frame := vmd.NewIkEnableFrame(0)
	frame.BoneName = boneName
	frame.Enabled = enabled
	return frame
}

// func computeInitialAllDeltas(
// 	sizingSet *domain.SizingSet, allFrames []int, blockSize int, model *pmx.PmxModel, motion *vmd.VmdMotion,
// ) (initialAllDeltas []*delta.VmdDeltas, err error) {
// 	initialMotion := vmd.NewVmdMotion("")
// 	centerBone, err := model.Bones.GetCenter()
// 	if err != nil {
// 		return nil, err
// 	}

// 	model.Bones.ForEach(func(index int, bone *pmx.Bone) bool {
// 		initialMotion.BoneFrames.Update(vmd.NewBoneNameFrames(bone.Name()))
// 		return true
// 	})

// 	for _, boneIndex := range centerBone.ParentBoneIndexes {
// 		bone, _ := model.Bones.Get(boneIndex)
// 		initialMotion.BoneFrames.Update(motion.BoneFrames.Get(bone.Name()))
// 	}

// 	if initialAllDeltas, err = computeVmdDeltas(allFrames, blockSize, model,
// 		initialMotion, sizingSet, false, gravity_bone_names, "足補正01"); err != nil {
// 		return nil, err
// 	}

// 	return initialAllDeltas, nil
// }

func createLowerIkBone(sizingSet *domain.SizingSet) *pmx.Bone {
	// 下半身IK
	lowerIkBone := pmx.NewBoneByName(fmt.Sprintf("%s%sIk", pmx.MLIB_PREFIX, pmx.LOWER.String()))
	lowerIkBone.Position = sizingSet.SizingLegCenterBone().Position.Copy()
	lowerIkBone.Ik = pmx.NewIk()
	lowerIkBone.Ik.BoneIndex = sizingSet.SizingLegCenterBone().Index()
	lowerIkBone.Ik.LoopCount = 100
	lowerIkBone.Ik.UnitRotation = &mmath.MVec3{X: 0.1, Y: 0.0, Z: 0.0}
	lowerIkBone.Ik.Links = make([]*pmx.IkLink, 1)
	{
		lowerIkBone.Ik.Links[0] = pmx.NewIkLink()
		lowerIkBone.Ik.Links[0].BoneIndex = sizingSet.SizingLowerBone().Index()
	}

	return lowerIkBone
}

// calculateAdjustedCenter は、センターおよびグルーブの位置補正を並列処理で計算します。
func calculateAdjustedLower(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int,
	originalAllDeltas, sizingAllDeltas []*delta.VmdDeltas,
	sizingProcessMotion *vmd.VmdMotion, incrementCompletedCount func(),
) error {
	// originalLowerPositions := make([]*mmath.MVec3, len(allFrames))
	// sizingLowerPositions := make([]*mmath.MVec3, len(allFrames))
	// legCenterPositions := make([]*mmath.MVec3, len(allFrames))
	// originalLowerXRotations := make([]*mmath.MQuaternion, len(allFrames))
	// originalLowerYRotations := make([]*mmath.MQuaternion, len(allFrames))
	// originalLowerZRotations := make([]*mmath.MQuaternion, len(allFrames))
	// sizingLowerZRotations := make([]*mmath.MQuaternion, len(allFrames))
	legCenterPositions := make([]*mmath.MVec3, len(allFrames))
	legCenterIdealPositions := make([]*mmath.MVec3, len(allFrames))
	originalLowerRotations := make([]*mmath.MQuaternion, len(allFrames))
	lowerRotations := make([]*mmath.MQuaternion, len(allFrames))
	lowerCancelRotations := make([]*mmath.MQuaternion, len(allFrames))
	// rightLegCancelRotations := make([]*mmath.MQuaternion, len(allFrames))

	// 体幹中心から足中心へのベクトル
	originalLowerVector := sizingSet.OriginalLegCenterBone().Position.Subed(sizingSet.OriginalTrunkRootBone().Position)
	sizingLowerVector := sizingSet.SizingLegCenterBone().Position.Subed(sizingSet.SizingTrunkRootBone().Position)
	lowerScale := &mmath.MVec3{X: 1.0, Y: sizingLowerVector.Length() / originalLowerVector.Length(), Z: 1.0}

	lowerIkBone := createLowerIkBone(sizingSet)

	err := miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			originalTrunkRootDelta := originalAllDeltas[index].Bones.GetByName(pmx.TRUNK_ROOT.String())
			originalLowerDelta := originalAllDeltas[index].Bones.GetByName(pmx.LOWER.String())
			originalLegCenterDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG_CENTER.String())
			// originalLeftLegDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG.StringFromDirection(pmx.BONE_DIRECTION_LEFT))
			// originalLeftAnkleDelta := originalAllDeltas[index].Bones.GetByName(pmx.ANKLE.StringFromDirection(pmx.BONE_DIRECTION_LEFT))
			// // originalRightLegDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG.StringFromDirection(pmx.BONE_DIRECTION_RIGHT))
			// originalRightAnkleDelta := originalAllDeltas[index].Bones.GetByName(pmx.ANKLE.StringFromDirection(pmx.BONE_DIRECTION_RIGHT))
			// originalLeftHeelDelta := originalAllDeltas[index].Bones.GetByName(pmx.HEEL.StringFromDirection(pmx.BONE_DIRECTION_LEFT))
			// originalRightHeelDelta := originalAllDeltas[index].Bones.GetByName(pmx.HEEL.StringFromDirection(pmx.BONE_DIRECTION_RIGHT))

			sizingTrunkRootDelta := sizingAllDeltas[index].Bones.GetByName(pmx.TRUNK_ROOT.String())
			// sizingLowerDelta := sizingAllDeltas[index].Bones.GetByName(sizingSet.SizingLowerBone().Name())
			// sizingLegCenterDelta := sizingAllDeltas[index].Bones.GetByName(sizingSet.SizingLowerBone().Name())
			// sizingLeftLegDelta := sizingAllDeltas[index].Bones.GetByName(pmx.LEG.StringFromDirection(pmx.BONE_DIRECTION_LEFT))
			// sizingLeftAnkleDelta := sizingAllDeltas[index].Bones.GetByName(pmx.ANKLE.StringFromDirection(pmx.BONE_DIRECTION_LEFT))
			// // sizingRightLegDelta := sizingAllDeltas[index].Bones.GetByName(pmx.LEG.StringFromDirection(pmx.BONE_DIRECTION_RIGHT))
			// sizingRightAnkleDelta := sizingAllDeltas[index].Bones.GetByName(pmx.ANKLE.StringFromDirection(pmx.BONE_DIRECTION_RIGHT))
			// sizingLeftHeelDelta := sizingAllDeltas[index].Bones.GetByName(pmx.HEEL.StringFromDirection(pmx.BONE_DIRECTION_LEFT))
			// sizingRightHeelDelta := sizingAllDeltas[index].Bones.GetByName(pmx.HEEL.StringFromDirection(pmx.BONE_DIRECTION_RIGHT))

			// 元の下半身から見た足中心のローカル位置
			originalLowerLocalPosition := originalLowerDelta.FilledGlobalMatrix().Inverted().MulVec3(originalLegCenterDelta.FilledGlobalPosition())
			// 足中心のローカル位置を先モデルのスケールに合わせる
			sizingLegCenterLocalPosition := originalLowerLocalPosition.Muled(lowerScale)
			// 元の下半身に先の足中心ローカル位置を合わせたグローバル位置
			sizingLegCenterOriginalPosition := originalLowerDelta.FilledGlobalMatrix().MulVec3(sizingLegCenterLocalPosition)
			// 元の体幹中心から見た、先の足中心のローカル位置
			sizingLegCenterIdealLocalPosition := originalTrunkRootDelta.FilledGlobalMatrix().Inverted().MulVec3(sizingLegCenterOriginalPosition)
			// 先の体幹中心から見た、先の足中心のグローバル位置
			sizingLegCenterIdealPosition := sizingTrunkRootDelta.FilledGlobalMatrix().MulVec3(sizingLegCenterIdealLocalPosition)

			sizingLowerDeltas := deform.DeformIk(sizingSet.SizingConfigModel, sizingProcessMotion, sizingAllDeltas[index], float32(data), lowerIkBone, sizingLegCenterIdealPosition, trunk_lower_bone_names, false)

			lowerRotations[index] = sizingLowerDeltas.Bones.GetByName(pmx.LOWER.String()).FilledFrameRotation()

			lowerBf := sizingProcessMotion.BoneFrames.Get(pmx.LOWER.String()).Get(float32(data))
			lowerCancelRotations[index] = lowerRotations[index].Inverted().Muled(lowerBf.FilledRotation()).Normalized()

			// // 元の足のベクトルと先の足のベクトルが合う様に、下半身の角度を調整する
			// originalHeelVector := originalRightHeelDelta.FilledGlobalPosition().Subed(originalLeftHeelDelta.FilledGlobalPosition()).Normalized()
			// sizingHeelVector := sizingRightHeelDelta.FilledGlobalPosition().Subed(sizingLeftHeelDelta.FilledGlobalPosition()).Normalized()

			// originalLowerVector := originalLegCenterDelta.FilledGlobalPosition().Subed(originalLowerDelta.FilledGlobalPosition()).Normalized()

			// lowerXRotation, lowerYRotation, lowerZRotation := lowerBf.FilledRotation().SeparateByAxis(originalLowerVector.Normalized())

			// lowerXRotation.Shorten()
			// lowerYRotation.Shorten()
			// lowerZRotation.Shorten()

			// sizingLowerZRotation := lowerZRotation.MuledScalar(lowerScale).Shorten()
			// lowerRotations[index] = sizingLowerZRotation.Muled(lowerYRotation).Muled(lowerXRotation).Normalized()

			// rightLegBf := sizingProcessMotion.BoneFrames.Get(pmx.LEG.StringFromDirection(pmx.BONE_DIRECTION_RIGHT)).Get(float32(data))
			// rightLegCancelRotations[index] = lowerRotations[index].Inverted().Muled(rightLegBf.FilledRotation()).Normalized()

			// originalAnkleCenterPosition := originalLeftAnkleDelta.FilledGlobalPosition().Added(originalRightAnkleDelta.FilledGlobalPosition()).MuledScalar(0.5)
			// originalLegVector := originalAnkleCenterPosition.Subed(originalLegCenterDelta.FilledGlobalPosition())

			// sizingLegCenterPosition := sizingLegCenterDelta.FilledGlobalPosition()
			// sizingAnkleCenterPosition := sizingLeftAnkleDelta.FilledGlobalPosition().Added(sizingRightAnkleDelta.FilledGlobalPosition()).MuledScalar(0.5)
			// sizingIdealAnkleCenterPosition := originalLegVector.Muled(moveScale).Added(sizingLegCenterDelta.FilledGlobalPosition())

			// sizingLegVector := sizingAnkleCenterPosition.Subed(sizingLegCenterPosition).Normalized()
			// sizingLegIdealVector := sizingIdealAnkleCenterPosition.Subed(sizingLegCenterPosition).Normalized()

			// lowerDiff := mmath.NewMQuaternionRotate(sizingLegVector, sizingLegIdealVector)

			// lowerBf := sizingProcessMotion.BoneFrames.Get(pmx.LOWER.String()).Get(float32(data))
			// lowerRotations[index] = lowerBf.FilledRotation().Muled(lowerDiff).Normalized()

			if mlog.IsDebug() {
				sizingLegCenterDelta := sizingAllDeltas[index].Bones.GetByName(pmx.LEG_CENTER.String())
				legCenterPositions[index] = sizingLegCenterDelta.FilledGlobalPosition()
				legCenterIdealPositions[index] = sizingLegCenterIdealPosition
				originalLowerRotations[index] = lowerBf.FilledRotation()

				// originalLowerPositions[index] = originalLowerDelta.FilledGlobalPosition()
				// sizingLowerDelta := sizingAllDeltas[index].Bones.GetByName(pmx.LOWER.String())
				// // sizingLegCenterDelta := sizingAllDeltas[index].Bones.GetByName(pmx.LEG_CENTER.String())
				// sizingLowerPositions[index] = sizingLowerDelta.FilledGlobalPosition()
				// // legCenterPositions[index] = sizingLegCenterDelta.FilledGlobalPosition()
				// // originalLowerXRotations[index] = lowerXRotation
				// // originalLowerYRotations[index] = lowerYRotation
				// // originalLowerZRotations[index] = lowerZRotation
				// // sizingLowerZRotations[index] = sizingLowerZRotation
				// // ankleCenterPositions[index] = sizingAnkleCenterPosition
				// // ankleCenterIdealPositions[index] = sizingIdealAnkleCenterPosition
			}

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正03", sizingSet.Index, iterIndex, allCount)
		})
	if err != nil {
		return err
	}

	if mlog.IsDebug() {
		motion := vmd.NewVmdMotion("")

		for i, iFrame := range allFrames {
			frame := float32(iFrame)
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = legCenterPositions[i]
				bf.Rotation = originalLowerRotations[i]
				motion.InsertBoneFrame("先足中心", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = legCenterIdealPositions[i]
				bf.Rotation = lowerRotations[i]
				motion.InsertBoneFrame("先理想足中心", bf)
			}
			// {
			// 	bf := vmd.NewBoneFrame(frame)
			// 	bf.Position = originalLowerPositions[i]
			// 	bf.Rotation = originalLowerYRotations[i]
			// 	motion.InsertBoneFrame("元下半身Y", bf)
			// }
			// {
			// 	bf := vmd.NewBoneFrame(frame)
			// 	bf.Position = originalLowerPositions[i]
			// 	bf.Rotation = originalLowerZRotations[i]
			// 	motion.InsertBoneFrame("元下半身Z", bf)
			// }
			// {
			// 	bf := vmd.NewBoneFrame(frame)
			// 	bf.Position = sizingLowerPositions[i]
			// 	bf.Rotation = sizingLowerZRotations[i]
			// 	motion.InsertBoneFrame("先下半身Z", bf)
			// }
			// {
			// 	bf := vmd.NewBoneFrame(frame)
			// 	bf.Position = sizingLowerPositions[i]
			// 	bf.Rotation = lowerRotations[i]
			// 	motion.InsertBoneFrame("先下半身", bf)
			// }
			// {
			// 	bf := vmd.NewBoneFrame(frame)
			// 	bf.Position = legCenterPositions[i]
			// 	motion.InsertBoneFrame("2足中心", bf)
			// }
			// {
			// 	bf := vmd.NewBoneFrame(frame)
			// 	bf.Position = ankleCenterPositions[i]
			// 	motion.InsertBoneFrame("2足首中心", bf)
			// }
			// {
			// 	bf := vmd.NewBoneFrame(frame)
			// 	bf.Position = ankleCenterIdealPositions[i]
			// 	motion.InsertBoneFrame("2理想足首中心", bf)
			// }
			// {
			// 	bf := vmd.NewBoneFrame(frame)
			// 	bf.Position = ankleCenterIdealPositions[i]
			// 	motion.InsertBoneFrame("2理想足首中心", bf)
			// }
		}

		outputVerboseMotion("足02", sizingSet.OutputMotionPath, motion)
	}

	incrementCompletedCount()

	// 下半身回転をサイジング先モーションに反映
	updateLower(sizingSet, allFrames, sizingProcessMotion, lowerRotations, lowerCancelRotations)

	return err
}

// updateLower は、補正した下半身回転をサイジング先モーションに反映します。
func updateLower(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	lowerRotations, lowerCancelRotations []*mmath.MQuaternion,
) {
	for i, iFrame := range allFrames {
		frame := float32(iFrame)
		{
			bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingLowerBone().Name()).Get(frame)
			bf.Rotation = lowerRotations[i]
			sizingProcessMotion.InsertBoneFrame(sizingSet.SizingLowerBone().Name(), bf)
		}
		{
			bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingLeftLegBone().Name()).Get(frame)
			bf.Rotation = lowerCancelRotations[i].Muled(bf.FilledRotation()).Normalized()
			sizingProcessMotion.InsertBoneFrame(sizingSet.SizingLeftLegBone().Name(), bf)
		}
		{
			bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingRightLegBone().Name()).Get(frame)
			bf.Rotation = lowerCancelRotations[i].Muled(bf.FilledRotation()).Normalized()
			sizingProcessMotion.InsertBoneFrame(sizingSet.SizingRightLegBone().Name(), bf)
		}

		if i > 0 && i%1000 == 0 {
			processLog("足補正04", sizingSet.Index, i, len(allFrames))
		}
	}

	if mlog.IsDebug() {
		outputVerboseMotion("足03", sizingSet.OutputMotionPath, sizingProcessMotion)
	}
}

// calculateAdjustedCenter は、センターおよびグルーブの位置補正を並列処理で計算します。
func calculateAdjustedCenter(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, moveScale *mmath.MVec3,
	originalAllDeltas, sizingAllDeltas []*delta.VmdDeltas,
	sizingProcessMotion *vmd.VmdMotion, incrementCompletedCount func(),
) error {
	centerPositions := make([]*mmath.MVec3, len(allFrames))
	groovePositions := make([]*mmath.MVec3, len(allFrames))

	originalGravities := make([]*mmath.MVec3, len(allFrames))
	sizingGravities := make([]*mmath.MVec3, len(allFrames))
	// originalLowestPositions := make([]*mmath.MVec3, len(allFrames))
	// sizingLowestPositions := make([]*mmath.MVec3, len(allFrames))
	gravityIdealPositions := make([]*mmath.MVec3, len(allFrames))
	// trunkRootIdealPositions := make([]*mmath.MVec3, len(allFrames))

	// // 先モデルの初期重心位置を計算
	// originalInitialVmdDeltas, _ := computeVmdDeltas([]int{0}, 1, sizingSet.OriginalConfigModel, vmd.InitialMotion, sizingSet, true, gravity_bone_names, "")
	// originalInitialGravityPos := calcGravity(originalInitialVmdDeltas[0])

	// // 元の足中心Yと重心Yの比率差を計算
	// gravityOriginalScaleDiff := originalInitialGravityPos.Dived(sizingSet.OriginalLegCenterBone().Position).Effective().One()

	// // 先モデルの初期重心位置を計算
	// sizingInitialVmdDeltas, _ := computeVmdDeltas([]int{0}, 1, sizingSet.SizingConfigModel, vmd.InitialMotion, sizingSet, true, gravity_bone_names, "")
	// sizingInitialGravityPos := calcGravity(sizingInitialVmdDeltas[0])

	// // 元と先の重心比率差を計算
	// gravityScale := sizingInitialGravityPos.Dived(originalInitialGravityPos).Effective().One()

	// // // 先の足中心Yと重心Yの比率差を計算
	// // gravitySizingScaleDiff := sizingInitialGravityPos.Dived(sizingSet.SizingLegCenterBone().Position).Effective().One()

	// // 元と先の足の全体の長さ比率差を計算
	// legScale := (sizingSet.SizingLegCenterBone().Position.Y - sizingSet.SizingLeftAnkleBone().Position.Y) /
	// 	(sizingSet.OriginalLegCenterBone().Position.Y - sizingSet.OriginalLeftAnkleBone().Position.Y)
	// ankleScale := sizingSet.SizingLeftAnkleBone().Position.Y / sizingSet.OriginalLeftAnkleBone().Position.Y
	// // 元の足中心Yのうち、足首が占める割合
	// originalAnkleRatio := sizingSet.OriginalLeftAnkleBone().Position.Y / sizingSet.OriginalLegCenterBone().Position.Y
	// // 先の足中心Yのうち、足-足首が占める割合
	// originalLegRatio := 1 - originalAnkleRatio
	// // 先の足中心Yのうち、足首が占める割合
	// sizingAnkleRatio := sizingSet.SizingLeftAnkleBone().Position.Y / sizingSet.SizingLegCenterBone().Position.Y
	// sizingLegRatio := 1 - sizingAnkleRatio

	// // 元と先の初期重心位置比率差を計算
	// gravityInitialScaleDiff := sizingInitialGravityPos.Dived(originalInitialGravityPos).Effective().One()

	// // 初期重心位置と体幹根元までの差分を計算
	// sizingInitialTrunkRootDiff := sizingSet.SizingTrunkRootBone().Position.Subed(sizingInitialGravityPos)

	// // センターの親ボーンまでの変形情報を保持したモーションを作成
	// originalInitialVmdDeltas, err := computeInitialAllDeltas(sizingSet, allFrames, blockSize, sizingSet.OriginalConfigModel, sizingProcessMotion)
	// sizingInitialAllDeltas, err := computeInitialAllDeltas(sizingSet, allFrames, blockSize, sizingSet.SizingConfigModel, sizingProcessMotion)

	// // 元と先の足中心から首根元までの長さ比率
	// originalUpperY := sizingSet.OriginalTrunkRootBone().Position.Y - sizingSet.OriginalLegCenterBone().Position.Y
	// sizingUpperY := sizingSet.SizingTrunkRootBone().Position.Y - sizingSet.SizingLegCenterBone().Position.Y
	// upperScale := sizingUpperY / originalUpperY

	// // 元と先の足の長さ比率
	// legScale := sizingSet.SizingLegCenterBone().Position.Y / sizingSet.OriginalLegCenterBone().Position.Y

	// 元モデルと先モデルの初期重心を計算
	originalInitialGravityPos := computeInitialGravity(sizingSet, sizingSet.OriginalConfigModel, vmd.InitialMotion)
	sizingInitialGravityPos := computeInitialGravity(sizingSet, sizingSet.SizingConfigModel, vmd.InitialMotion)
	gravityScale := sizingInitialGravityPos.Y / originalInitialGravityPos.Y
	isActiveGroove := false
	sizingProcessMotion.BoneFrames.Get(pmx.GROOVE.String()).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
		if !mmath.NearEquals(bf.FilledPosition().Y, 0.0, 1e-3) {
			isActiveGroove = true
			return false
		}
		return true
	})

	// totalGravityScale := gravityScale * legScale

	sizingCenterParentBone := sizingSet.SizingCenterBone().ParentBone

	err := miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			// // 元の最も地面に近い（Y=0に近い）ボーンを取得
			// originalLowestBonePosition := mmath.MVec3MaxVal
			// var originalLowestBone *pmx.Bone
			// sizingSet.OriginalConfigModel.Bones.ForEach(func(index int, bone *pmx.Bone) bool {
			// 	if originalAllDeltas[index].Bones.Contains(bone.Index()) &&
			// 		sizingAllDeltas[index].Bones.ContainsByName(bone.Name()) &&
			// 		originalLowestBonePosition.Y >= originalAllDeltas[index].Bones.Get(bone.Index()).FilledGlobalPosition().Y {
			// 		originalLowestBonePosition = originalAllDeltas[index].Bones.Get(bone.Index()).FilledGlobalPosition()
			// 		originalLowestBone = bone
			// 	}
			// 	return true
			// })

			// sizingLowestBone := sizingAllDeltas[index].Bones.GetByName(originalLowestBone.Name())
			// sizingLowestBonePosition := sizingLowestBone.FilledGlobalPosition()

			// // 元の最も地面に近い（Y=0に近い）ボーンの高さを元に、先のボーンの高さを計算
			// sizingOffsetY := (originalLowestBonePosition.Y * moveScale.Y) - sizingLowestBonePosition.Y

			originalGravityPos := calcGravity(originalAllDeltas[index])
			sizingGravityPos := calcGravity(sizingAllDeltas[index])
			sizingFixCenterTargetY := originalGravityPos.Y * gravityScale

			gravityIdealPosition := sizingGravityPos.Copy()
			gravityIdealPosition.Y = sizingFixCenterTargetY

			sizingCenterParentDelta := sizingAllDeltas[index].Bones.GetByName(sizingCenterParentBone.Name())

			// センターの親から見た重心のローカル位置
			sizingGravityLocalPosition := sizingCenterParentDelta.FilledGlobalMatrix().Inverted().MulVec3(sizingGravityPos)

			// センターの親から見た理想の重心のローカル位置
			sizingGravityIdealLocalPosition := sizingCenterParentDelta.FilledGlobalMatrix().Inverted().MulVec3(gravityIdealPosition)

			// 重心の差分
			sizingGravityDiff := sizingGravityIdealLocalPosition.Subed(sizingGravityLocalPosition)

			// if mlog.IsDebug() {
			// 	originalGravities[index] = originalGravityPos
			// 	sizingGravities[index] = sizingGravityPos

			// 	centerIdealPositions[index] = gravityIdealPosition
			// }

			// centerBf := sizingProcessMotion.BoneFrames.Get(pmx.CENTER.String()).Get(float32(data))
			// centerPositions[index] = centerBf.Position.Muled(moveScale)
			// centerPositions[index].Y = centerBf.FilledPosition().Added(centerIdealPositions[index].Subed(sizingGravityPos)).Y
			// // 元と先の重心比率差を計算
			// gravityScaleDiff := sizingGravityPos.Dived(originalGravityPos).Effective()

			// // 元モデルと先モデルの初期重心を計算
			// originalInitialGravityPos := calcGravity(originalInitialVmdDeltas[index])
			// sizingInitialGravityPos := calcGravity(sizingInitialAllDeltas[index])

			// // 元の重心比率差を計算
			// gravityDiff := originalGravityPos.Dived(originalInitialGravityPos).Effective()

			// // 元の重心のうち、足と足首がそれぞれ占める割合
			// originalLegLength := originalGravityPos.Y * originalLegRatio
			// originalAnkleLength := originalGravityPos.Y * originalAnkleRatio

			// // 元の長さを、先の長さに適用
			// sizingLegLength := originalLegLength * legScale * moveScale.X
			// sizingAnkleLength := originalAnkleLength * ankleScale * moveScale.X

			// // 理想の重心Y位置を計算
			// gravityIdealPosition := &mmath.MVec3{
			// 	X: sizingGravityPos.X,
			// 	Y: sizingGravityPos.Y + sizingOffsetY,
			// 	Z: sizingGravityPos.Z,
			// }

			centerBf := sizingProcessMotion.BoneFrames.Get(pmx.CENTER.String()).Get(float32(data))
			grooveBf := sizingProcessMotion.BoneFrames.Get(pmx.GROOVE.String()).Get(float32(data))

			centerPosition := centerBf.FilledPosition().Added(grooveBf.FilledPosition())
			centerPositions[index] = centerPosition.Muled(moveScale)
			centerPositions[index].Y = centerPosition.Y + sizingGravityDiff.Y

			// sizingTrunkRootMat := sizingAllDeltas[index].Bones.Get(sizingSet.SizingTrunkRootBone().Index()).FilledGlobalMatrix()

			// trunkRootIdealPosition := sizingTrunkRootMat.MulVec3(&mmath.MVec3{X: 0, Y: sizingOffsetY, Z: 0})
			// centerPositions[index].Y = centerPosition.Y +
			// 	sizingTrunkRootMat.Inverted().MulVec3(trunkRootIdealPosition).Y

			if mlog.IsDebug() {
				// originalLowestPositions[index] = originalLowestBonePosition
				// sizingLowestPositions[index] = sizingLowestBonePosition
				originalGravities[index] = originalGravityPos
				sizingGravities[index] = sizingGravityPos
				gravityIdealPositions[index] = gravityIdealPosition
				// trunkRootIdealPositions[index] = trunkRootIdealPosition
			}

			if isActiveGroove {
				// グルーブがある場合、Yをグルーブ位置に移動
				groovePositions[index] = &mmath.MVec3{X: 0, Y: centerPositions[index].Y, Z: 0}
				centerPositions[index].Y = 0
			}

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正05", sizingSet.Index, iterIndex, allCount)
		})
	if err != nil {
		return err
	}

	if mlog.IsDebug() {
		motion := vmd.NewVmdMotion("")

		for i, iFrame := range allFrames {
			frame := float32(iFrame)
			// {
			// 	bf := vmd.NewBoneFrame(frame)
			// 	bf.Position = originalLowestPositions[i]
			// 	motion.InsertBoneFrame("元底辺", bf)
			// }
			// {
			// 	bf := vmd.NewBoneFrame(frame)
			// 	bf.Position = sizingLowestPositions[i]
			// 	motion.InsertBoneFrame("先底辺", bf)
			// }
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = originalGravities[i]
				motion.InsertBoneFrame("元重心", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = sizingGravities[i]
				motion.InsertBoneFrame("先重心", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = gravityIdealPositions[i]
				motion.InsertBoneFrame("理想重心", bf)
			}
			// {
			// 	bf := vmd.NewBoneFrame(frame)
			// 	bf.Position = trunkRootIdealPositions[i]
			// 	motion.InsertBoneFrame("理想体幹根元", bf)
			// }
			{
				bf := vmd.NewBoneFrame(frame)
				if groovePositions[i] == nil {
					bf.Position = centerPositions[i]
				} else {
					bf.Position = centerPositions[i].Added(groovePositions[i])
				}
				motion.InsertBoneFrame("重心センター", bf)
			}
		}

		outputVerboseMotion("足04", sizingSet.OutputMotionPath, motion)
	}

	incrementCompletedCount()

	// センター・グルーブ位置をサイジング先モーションに反映
	updateCenter(sizingSet, allFrames, sizingProcessMotion, centerPositions, groovePositions)

	return nil
}

// updateCenter は、計算したセンター・グルーブ位置をサイジング先モーションに反映します。
func updateCenter(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	centerPositions, groovePositions []*mmath.MVec3,
) {
	for i, iFrame := range allFrames {
		frame := float32(iFrame)
		sizingCenterBf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingCenterBone().Name()).Get(frame)
		sizingCenterBf.Position = centerPositions[i]
		sizingProcessMotion.InsertBoneFrame(sizingSet.SizingCenterBone().Name(), sizingCenterBf)

		if sizingSet.SizingGrooveVanillaBone() != nil {
			sizingGrooveBf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingGrooveBone().Name()).Get(frame)
			sizingGrooveBf.Position = groovePositions[i]
			sizingProcessMotion.InsertBoneFrame(sizingSet.SizingGrooveBone().Name(), sizingGrooveBf)
		}

		if i > 0 && i%1000 == 0 {
			processLog("足補正06", sizingSet.Index, i, len(allFrames))
		}
	}

	if mlog.IsDebug() {
		outputVerboseMotion("足05", sizingSet.OutputMotionPath, sizingProcessMotion)
	}
}

// calculateAdjustedLegIK は、足IK 補正の計算を並列処理で行い、各フレームごとの位置・回転補正値を算出します。
func calculateAdjustedLegIK(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, moveScale *mmath.MVec3,
	originalAllDeltas, sizingOffAllDeltas []*delta.VmdDeltas, sizingProcessMotion *vmd.VmdMotion,
	incrementCompletedCount func(),
) (leftLegAnkleIdealPositions, rightLegAnkleIdealPositions []*mmath.MVec3, err error) {

	leftLegIkPositions := make([]*mmath.MVec3, len(allFrames))
	leftLegIkRotations := make([]*mmath.MQuaternion, len(allFrames))
	rightLegIkPositions := make([]*mmath.MVec3, len(allFrames))
	rightLegIkRotations := make([]*mmath.MQuaternion, len(allFrames))
	leftLegAnkleIdealPositions = make([]*mmath.MVec3, len(allFrames))
	rightLegAnkleIdealPositions = make([]*mmath.MVec3, len(allFrames))

	leftLegBeforePositions := make([]*mmath.MVec3, len(allFrames))
	rightLegBeforePositions := make([]*mmath.MVec3, len(allFrames))
	leftToeBeforePositions := make([]*mmath.MVec3, len(allFrames))
	rightToeBeforePositions := make([]*mmath.MVec3, len(allFrames))
	leftHeelBeforePositions := make([]*mmath.MVec3, len(allFrames))
	rightHeelBeforePositions := make([]*mmath.MVec3, len(allFrames))
	leftToeIdealPositions := make([]*mmath.MVec3, len(allFrames))
	rightToeIdealPositions := make([]*mmath.MVec3, len(allFrames))
	leftHeelIdealPositions := make([]*mmath.MVec3, len(allFrames))
	rightHeelIdealPositions := make([]*mmath.MVec3, len(allFrames))
	leftLegIdealPositions := make([]*mmath.MVec3, len(allFrames))
	rightLegIdealPositions := make([]*mmath.MVec3, len(allFrames))
	leftLegIkInitialPositions := make([]*mmath.MVec3, len(allFrames))
	rightLegIkInitialPositions := make([]*mmath.MVec3, len(allFrames))
	leftLegIkParentPositions := make([]*mmath.MVec3, len(allFrames))
	rightLegIkParentPositions := make([]*mmath.MVec3, len(allFrames))

	originalLeftDiff := sizingSet.OriginalLeftToeTailBone().Position.Subed(sizingSet.OriginalLeftLegIkBone().Position)
	sizingLeftDiff := sizingSet.SizingLeftToeTailBone().Position.Subed(sizingSet.SizingLeftLegIkBone().Position)
	leftAnkleScale := sizingLeftDiff.Dived(originalLeftDiff).Absed()
	leftAnkleScale.X = 1.0

	originalRightDiff := sizingSet.OriginalRightToeTailBone().Position.Subed(sizingSet.OriginalRightLegIkBone().Position)
	sizingRightDiff := sizingSet.SizingRightToeTailBone().Position.Subed(sizingSet.SizingRightLegIkBone().Position)
	rightAnkleScale := sizingRightDiff.Dived(originalRightDiff).Absed()
	rightAnkleScale.X = 1.0

	leftLegIkDirectionSlope := sizingSet.SizingLeftToeTailBone().Position.Subed(sizingSet.SizingLeftAnkleBone().Position).Normalize()
	leftLegIkUpSlope := sizingSet.SizingLeftToeTailBone().Position.Subed(sizingSet.SizingLeftToePBone().Position).Normalize()
	leftLegIkCrossSlope := leftLegIkUpSlope.Cross(leftLegIkDirectionSlope).Normalize()
	leftLegIkSlopeQuat := mmath.NewMQuaternionFromDirection(leftLegIkDirectionSlope, leftLegIkCrossSlope)

	rightLegIkDirectionSlope := sizingSet.SizingRightToeTailBone().Position.Subed(sizingSet.SizingRightAnkleBone().Position).Normalize()
	rightLegIkUpSlope := sizingSet.SizingRightToeTailBone().Position.Subed(sizingSet.SizingRightToePBone().Position).Normalize()
	rightLegIkCrossSlope := rightLegIkUpSlope.Cross(rightLegIkDirectionSlope).Normalize()
	rightLegIkSlopeQuat := mmath.NewMQuaternionFromDirection(rightLegIkDirectionSlope, rightLegIkCrossSlope)

	sizingLeftLegIkParentBone, _ := sizingSet.SizingConfigModel.Bones.Get(sizingSet.SizingLeftLegIkBone().ParentIndex)
	sizingRightLegIkParentBone, _ := sizingSet.SizingConfigModel.Bones.Get(sizingSet.SizingRightLegIkBone().ParentIndex)

	// leftLegIkDiff := sizingSet.SizingLeftLegIkBone().Position.Subed(sizingLeftLegIkParentBone.Position)
	// rightLegIkDiff := sizingSet.SizingRightLegIkBone().Position.Subed(sizingRightLegIkParentBone.Position)

	// leftLegIkDiff := sizingLeftLegIkParentBone.Position.Subed(sizingSet.SizingLeftLegIkBone().Position)
	// rightLegIkDiff := sizingRightLegIkParentBone.Position.Subed(sizingSet.SizingRightLegIkBone().Position)

	err = miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			// サイジング先モデルの各足ボーンのデフォーム結果取得
			sizingLeftAnkleDelta := sizingOffAllDeltas[index].Bones.GetByName(pmx.ANKLE.Left())
			sizingLeftToePDelta := sizingOffAllDeltas[index].Bones.GetByName(pmx.TOE_P.Left())
			sizingLeftToeTailDelta := sizingOffAllDeltas[index].Bones.GetByName(pmx.TOE_T.Left())
			sizingRightAnkleDelta := sizingOffAllDeltas[index].Bones.GetByName(pmx.ANKLE.Right())
			sizingRightToePDelta := sizingOffAllDeltas[index].Bones.GetByName(pmx.TOE_P.Right())
			sizingRightToeTailDelta := sizingOffAllDeltas[index].Bones.GetByName(pmx.TOE_T.Right())

			// 足IK の回転(Dを見ない)
			leftLegFkDirectionSlope := sizingLeftToeTailDelta.FilledGlobalPosition().Subed(
				sizingLeftAnkleDelta.FilledGlobalPosition()).Normalize()
			leftLegFkUpSlope := sizingLeftToeTailDelta.FilledGlobalPosition().Subed(
				sizingLeftToePDelta.FilledGlobalPosition()).Normalize()
			leftLegCrossSlope := leftLegFkUpSlope.Cross(leftLegFkDirectionSlope).Normalize()
			leftLegFkSlopeQuat := mmath.NewMQuaternionFromDirection(leftLegFkDirectionSlope, leftLegCrossSlope)
			leftLegIkRotations[index] = leftLegFkSlopeQuat.Muled(leftLegIkSlopeQuat.Inverted())

			rightLegFkDirectionSlope := sizingRightToeTailDelta.FilledGlobalPosition().Subed(
				sizingRightAnkleDelta.FilledGlobalPosition()).Normalize()
			rightLegFkUpSlope := sizingRightToeTailDelta.FilledGlobalPosition().Subed(
				sizingRightToePDelta.FilledGlobalPosition()).Normalize()
			rightLegCrossSlope := rightLegFkUpSlope.Cross(rightLegFkDirectionSlope).Normalize()
			rightLegFkSlopeQuat := mmath.NewMQuaternionFromDirection(rightLegFkDirectionSlope, rightLegCrossSlope)
			rightLegIkRotations[index] = rightLegFkSlopeQuat.Muled(rightLegIkSlopeQuat.Inverted())

			// 足Yを求める為、D系ボーンを取得
			originalLeftHeelDDelta := originalAllDeltas[index].Bones.GetByName(pmx.HEEL.Left())
			originalLeftToeTailDDelta := originalAllDeltas[index].Bones.GetByName(pmx.TOE_T.Left())
			originalRightHeelDDelta := originalAllDeltas[index].Bones.GetByName(pmx.HEEL_D.Right())
			originalRightToeTailDDelta := originalAllDeltas[index].Bones.GetByName(pmx.TOE_T_D.Right())
			sizingLeftHeelDDelta := sizingOffAllDeltas[index].Bones.GetByName(pmx.HEEL_D.Left())
			sizingLeftToeTailDDelta := sizingOffAllDeltas[index].Bones.GetByName(pmx.TOE_T_D.Left())
			sizingRightHeelDDelta := sizingOffAllDeltas[index].Bones.GetByName(pmx.HEEL_D.Right())
			sizingRightToeTailDDelta := sizingOffAllDeltas[index].Bones.GetByName(pmx.TOE_T_D.Right())

			// 足IK の Y 軸補正
			leftLegIkAdjustedY := calcLegIkPositionY(index, sizingSet.OriginalLeftAnkleBone(),
				originalLeftToeTailDDelta, originalLeftHeelDDelta,
				sizingLeftToeTailDDelta, sizingLeftHeelDDelta, moveScale,
				leftToeBeforePositions, leftToeIdealPositions, leftHeelBeforePositions, leftHeelIdealPositions)
			rightLegIkAdjustedY := calcLegIkPositionY(index, sizingSet.OriginalRightAnkleBone(),
				originalRightToeTailDDelta, originalRightHeelDDelta,
				sizingRightToeTailDDelta, sizingRightHeelDDelta, moveScale,
				rightToeBeforePositions, rightToeIdealPositions, rightHeelBeforePositions, rightHeelIdealPositions)

			// 足IK の理想位置計算 （足首位置から各IKボーンの親へのオフセット）
			leftLegAnkleIdealPosition := sizingLeftAnkleDelta.FilledGlobalPosition().Copy()
			leftLegAnkleIdealPosition.Y += leftLegIkAdjustedY
			leftLegAnkleIdealPositions[index] = leftLegAnkleIdealPosition

			rightLegAnkleIdealPosition := sizingRightAnkleDelta.FilledGlobalPosition().Copy()
			rightLegAnkleIdealPosition.Y += rightLegIkAdjustedY
			rightLegAnkleIdealPositions[index] = rightLegAnkleIdealPosition

			sizingLeftLegIkDelta := sizingOffAllDeltas[index].Bones.GetByName(pmx.LEG_IK.Left())
			sizingRightLegIkDelta := sizingOffAllDeltas[index].Bones.GetByName(pmx.LEG_IK.Right())
			sizingLeftLegIkParentDelta := sizingOffAllDeltas[index].Bones.Get(sizingLeftLegIkParentBone.Index())
			sizingRightLegIkParentDelta := sizingOffAllDeltas[index].Bones.Get(sizingRightLegIkParentBone.Index())

			// sizingLeftLegIkInitialMatrix := sizingLeftLegIkParentDelta.FilledGlobalMatrix().Translated(leftLegIkDiff)
			// sizingRightLegIkInitialMatrix := sizingRightLegIkParentDelta.FilledGlobalMatrix().Translated(rightLegIkDiff)

			// 足IKの親から見た足IKの相対位置
			leftLegIkInitialPosition := sizingLeftLegIkParentDelta.FilledGlobalMatrix().Inverted().MulVec3(sizingLeftLegIkDelta.FilledGlobalPosition())
			rightLegIkInitialPosition := sizingRightLegIkParentDelta.FilledGlobalMatrix().Inverted().MulVec3(sizingRightLegIkDelta.FilledGlobalPosition())

			// 足IKの親から見た理想足首の相対位置
			leftIdealAnklePosition := sizingLeftLegIkParentDelta.FilledGlobalMatrix().Inverted().MulVec3(leftLegAnkleIdealPosition)
			rightIdealAnklePosition := sizingRightLegIkParentDelta.FilledGlobalMatrix().Inverted().MulVec3(rightLegAnkleIdealPosition)

			// 現在の足IKの位置と理想足首位置の差分
			leftLegIkPositionDiff := leftIdealAnklePosition.Subed(leftLegIkInitialPosition)
			rightLegIkPositionDiff := rightIdealAnklePosition.Subed(rightLegIkInitialPosition)

			sizingLeftLegIkBf := sizingProcessMotion.BoneFrames.Get(pmx.LEG_IK.Left()).Get(float32(data))
			sizingRightLegIkBf := sizingProcessMotion.BoneFrames.Get(pmx.LEG_IK.Right()).Get(float32(data))

			leftLegIkPositions[index] = sizingLeftLegIkBf.FilledPosition().Added(leftLegIkPositionDiff)
			if mmath.NearEquals(sizingLeftLegIkBf.FilledPosition().X, 0.0, 1e-3) {
				leftLegIkPositions[index].X = sizingLeftLegIkBf.FilledPosition().X
			}
			if mmath.NearEquals(sizingLeftLegIkBf.FilledPosition().Y, 0.0, 1e-3) {
				leftLegIkPositions[index].Y = sizingLeftLegIkBf.FilledPosition().Y
			}
			if mmath.NearEquals(sizingLeftLegIkBf.FilledPosition().Z, 0.0, 1e-3) {
				leftLegIkPositions[index].Z = sizingLeftLegIkBf.FilledPosition().Z
			}
			rightLegIkPositions[index] = sizingRightLegIkBf.FilledPosition().Added(rightLegIkPositionDiff)
			if mmath.NearEquals(sizingRightLegIkBf.FilledPosition().X, 0.0, 1e-3) {
				rightLegIkPositions[index].X = sizingRightLegIkBf.FilledPosition().X
			}
			if mmath.NearEquals(sizingRightLegIkBf.FilledPosition().Y, 0.0, 1e-3) {
				rightLegIkPositions[index].Y = sizingRightLegIkBf.FilledPosition().Y
			}
			if mmath.NearEquals(sizingRightLegIkBf.FilledPosition().Z, 0.0, 1e-3) {
				rightLegIkPositions[index].Z = sizingRightLegIkBf.FilledPosition().Z
			}

			if mlog.IsDebug() {
				leftLegBeforePositions[index] = sizingLeftAnkleDelta.FilledGlobalPosition().Copy()
				rightLegBeforePositions[index] = sizingRightAnkleDelta.FilledGlobalPosition().Copy()
				leftLegIdealPositions[index] = leftLegAnkleIdealPosition
				rightLegIdealPositions[index] = rightLegAnkleIdealPosition
				leftLegIkParentPositions[index] = sizingLeftLegIkParentDelta.FilledGlobalPosition()
				rightLegIkParentPositions[index] = sizingRightLegIkParentDelta.FilledGlobalPosition()
				leftLegIkInitialPositions[index] = sizingLeftLegIkDelta.FilledGlobalPosition()
				rightLegIkInitialPositions[index] = sizingRightLegIkDelta.FilledGlobalPosition()
			}

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正07", sizingSet.Index, iterIndex, allCount)
		})
	if err != nil {
		return nil, nil, err
	}

	if mlog.IsDebug() {
		motion := vmd.NewVmdMotion("")

		for i, iFrame := range allFrames {
			frame := float32(iFrame)
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = leftLegIkParentPositions[i]
				motion.InsertBoneFrame("左IK親初期", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = rightLegIkParentPositions[i]
				motion.InsertBoneFrame("右IK親初期", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = leftLegBeforePositions[i]
				motion.InsertBoneFrame("左足IK補正前", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = rightLegBeforePositions[i]
				motion.InsertBoneFrame("右足IK補正前", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = leftLegIdealPositions[i]
				motion.InsertBoneFrame("左足IK理想", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = rightLegIdealPositions[i]
				motion.InsertBoneFrame("右足IK理想", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = leftLegIkInitialPositions[i]
				motion.InsertBoneFrame("左足IK初期", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = rightLegIkInitialPositions[i]
				motion.InsertBoneFrame("右足IK初期", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = leftLegIkPositions[i]
				motion.InsertBoneFrame("左足IK補正後", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = rightLegIkPositions[i]
				motion.InsertBoneFrame("右足IK補正後", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = leftToeBeforePositions[i]
				motion.InsertBoneFrame("左つま先補正前", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = leftToeIdealPositions[i]
				motion.InsertBoneFrame("左つま先理想", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = rightToeBeforePositions[i]
				motion.InsertBoneFrame("右つま先補正前", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = rightToeIdealPositions[i]
				motion.InsertBoneFrame("右つま先理想", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = leftHeelBeforePositions[i]
				motion.InsertBoneFrame("左かかと補正前", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = leftHeelIdealPositions[i]
				motion.InsertBoneFrame("左かかと理想", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = rightHeelBeforePositions[i]
				motion.InsertBoneFrame("右かかと補正前", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = rightHeelIdealPositions[i]
				motion.InsertBoneFrame("右かかと理想", bf)
			}
		}

		outputVerboseMotion("足06", sizingSet.OutputMotionPath, motion)
	}

	incrementCompletedCount()

	// 足IK 補正値をサイジング先モーションに反映
	updateLegIK(sizingSet, allFrames, sizingProcessMotion,
		leftLegIkPositions, leftLegIkRotations, rightLegIkPositions, rightLegIkRotations, originalAllDeltas)

	return leftLegAnkleIdealPositions, rightLegAnkleIdealPositions, nil
}

// updateLegIK は、計算済みの足IK 補正位置と回転値をモーションデータに反映させます。
func updateLegIK(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	leftLegIkPositions []*mmath.MVec3, leftLegIkRotations []*mmath.MQuaternion,
	rightLegIkPositions []*mmath.MVec3, rightLegIkRotations []*mmath.MQuaternion,
	originalAllDeltas []*delta.VmdDeltas,
) {
	inheritanceLeftPositions := make([]*mmath.MVec3, len(allFrames))
	inheritanceRightPositions := make([]*mmath.MVec3, len(allFrames))

	for i, iFrame := range allFrames {
		if i > 0 {
			// 前フレームと同じ足首位置なら前フレームの値を継承
			originalLeftAnklePosition := originalAllDeltas[i].Bones.GetByName(pmx.ANKLE.Left()).FilledGlobalPosition()
			originalRightAnklePosition := originalAllDeltas[i].Bones.GetByName(pmx.ANKLE.Right()).FilledGlobalPosition()
			originalLeftAnklePrevPosition := originalAllDeltas[i-1].Bones.GetByName(pmx.ANKLE.Left()).FilledGlobalPosition()
			originalRightAnklePrevPosition := originalAllDeltas[i-1].Bones.GetByName(pmx.ANKLE.Right()).FilledGlobalPosition()

			if mmath.NearEquals(originalLeftAnklePrevPosition.X, originalLeftAnklePosition.X, 1e-3) {
				leftLegIkPositions[i].X = leftLegIkPositions[i-1].X
				if mlog.IsDebug() {
					if inheritanceLeftPositions[i] == nil {
						inheritanceLeftPositions[i] = mmath.NewMVec3()
					}
					inheritanceLeftPositions[i].X = leftLegIkPositions[i].X
				}
			}
			if mmath.NearEquals(originalLeftAnklePrevPosition.Y, originalLeftAnklePosition.Y, 1e-3) {
				leftLegIkPositions[i].Y = leftLegIkPositions[i-1].Y
				if mlog.IsDebug() {
					if inheritanceLeftPositions[i] == nil {
						inheritanceLeftPositions[i] = mmath.NewMVec3()
					}
					inheritanceLeftPositions[i].Y = leftLegIkPositions[i].Y
				}
			}
			if mmath.NearEquals(originalLeftAnklePrevPosition.Z, originalLeftAnklePosition.Z, 1e-3) {
				leftLegIkPositions[i].Z = leftLegIkPositions[i-1].Z
				if mlog.IsDebug() {
					if inheritanceLeftPositions[i] == nil {
						inheritanceLeftPositions[i] = mmath.NewMVec3()
					}
					inheritanceLeftPositions[i].Z = leftLegIkPositions[i].Z
				}
			}
			if mmath.NearEquals(originalRightAnklePrevPosition.X, originalRightAnklePosition.X, 1e-3) {
				rightLegIkPositions[i].X = rightLegIkPositions[i-1].X
				if mlog.IsDebug() {
					if inheritanceRightPositions[i] == nil {
						inheritanceRightPositions[i] = mmath.NewMVec3()
					}
					inheritanceRightPositions[i].X = rightLegIkPositions[i].X
				}
			}
			if mmath.NearEquals(originalRightAnklePrevPosition.Y, originalRightAnklePosition.Y, 1e-3) {
				rightLegIkPositions[i].Y = rightLegIkPositions[i-1].Y
				if mlog.IsDebug() {
					if inheritanceRightPositions[i] == nil {
						inheritanceRightPositions[i] = mmath.NewMVec3()
					}
					inheritanceRightPositions[i].Y = rightLegIkPositions[i].Y
				}
			}
			if mmath.NearEquals(originalRightAnklePrevPosition.Z, originalRightAnklePosition.Z, 1e-3) {
				rightLegIkPositions[i].Z = rightLegIkPositions[i-1].Z
				if mlog.IsDebug() {
					if inheritanceRightPositions[i] == nil {
						inheritanceRightPositions[i] = mmath.NewMVec3()
					}
					inheritanceRightPositions[i].Z = rightLegIkPositions[i].Z
				}
			}
		}

		frame := float32(iFrame)
		// 各フレームの IK値を設定
		rightLegIkBf := sizingProcessMotion.BoneFrames.Get(pmx.LEG_IK.Right()).Get(frame)
		rightLegIkBf.Position = rightLegIkPositions[i]
		rightLegIkBf.Rotation = rightLegIkRotations[i]
		sizingProcessMotion.InsertBoneFrame(pmx.LEG_IK.Right(), rightLegIkBf)

		leftLegIkBf := sizingProcessMotion.BoneFrames.Get(pmx.LEG_IK.Left()).Get(frame)
		leftLegIkBf.Position = leftLegIkPositions[i]
		leftLegIkBf.Rotation = leftLegIkRotations[i]
		sizingProcessMotion.InsertBoneFrame(pmx.LEG_IK.Left(), leftLegIkBf)

		if i > 0 && i%1000 == 0 {
			processLog("足補正08", sizingSet.Index, i, len(allFrames))
		}
	}

	if mlog.IsDebug() {
		outputVerboseMotion("足07", sizingSet.OutputMotionPath, sizingProcessMotion)
	}

	if mlog.IsDebug() {
		motion := vmd.NewVmdMotion("")

		for i, iFrame := range allFrames {
			frame := float32(iFrame)
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = inheritanceLeftPositions[i]
				motion.InsertBoneFrame("左足継承", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = inheritanceRightPositions[i]
				motion.InsertBoneFrame("右足継承", bf)
			}
		}

		outputVerboseMotion("足08", sizingSet.OutputMotionPath, motion)
	}
}

// calculateAdjustedLegFK は、IK ON状態で足FK の回転を再計算し、その結果を配列で返します。
func calculateAdjustedLegFK(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int,
	sizingOffAllDeltas []*delta.VmdDeltas, leftLegAnkleIdealPositions, rightLegAnkleIdealPositions []*mmath.MVec3,
	sizingProcessMotion *vmd.VmdMotion, incrementCompletedCount func(),
) error {
	leftLegRotations := make([]*mmath.MQuaternion, len(allFrames))
	leftKneeRotations := make([]*mmath.MQuaternion, len(allFrames))
	leftAnkleRotations := make([]*mmath.MQuaternion, len(allFrames))
	rightLegRotations := make([]*mmath.MQuaternion, len(allFrames))
	rightKneeRotations := make([]*mmath.MQuaternion, len(allFrames))
	rightAnkleRotations := make([]*mmath.MQuaternion, len(allFrames))

	err := miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			sizingLeftLegDeltas := deform.DeformIk(sizingSet.SizingConfigModel, sizingProcessMotion,
				sizingOffAllDeltas[index], float32(data), sizingSet.SizingLeftLegIkBone(),
				leftLegAnkleIdealPositions[index], leg_direction_bone_names[0], false)

			leftLegRotations[index] = sizingLeftLegDeltas.Bones.GetByName(pmx.LEG.Left()).FilledFrameRotation()
			leftKneeRotations[index] = sizingLeftLegDeltas.Bones.GetByName(pmx.KNEE.Left()).FilledFrameRotation()
			leftAnkleRotations[index] = sizingLeftLegDeltas.Bones.GetByName(pmx.ANKLE.Left()).FilledFrameRotation()

			sizingRightLegDeltas := deform.DeformIk(sizingSet.SizingConfigModel, sizingProcessMotion,
				sizingOffAllDeltas[index], float32(data), sizingSet.SizingRightLegIkBone(),
				rightLegAnkleIdealPositions[index], leg_direction_bone_names[1], false)

			rightLegRotations[index] = sizingRightLegDeltas.Bones.GetByName(pmx.LEG.Right()).FilledFrameRotation()
			rightKneeRotations[index] = sizingRightLegDeltas.Bones.GetByName(pmx.KNEE.Right()).FilledFrameRotation()
			rightAnkleRotations[index] = sizingRightLegDeltas.Bones.GetByName(pmx.ANKLE.Right()).FilledFrameRotation()
			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正09", sizingSet.Index, iterIndex, allCount)
		})
	if err != nil {
		return err
	}

	incrementCompletedCount()

	// 再計算した回転情報をサイジング先モーションに反映
	updateAdjustedLegFk(sizingSet, allFrames, sizingProcessMotion,
		leftLegRotations, leftKneeRotations, leftAnkleRotations,
		rightLegRotations, rightKneeRotations, rightAnkleRotations)

	return nil
}

func updateAdjustedLegFk(
	sizingSet *domain.SizingSet, allFrames []int,
	sizingProcessMotion *vmd.VmdMotion,
	leftLegRotations, leftKneeRotations, leftAnkleRotations,
	rightLegRotations, rightKneeRotations, rightAnkleRotations []*mmath.MQuaternion,
) {

	// サイジング先にFKを焼き込み
	for i, iFrame := range allFrames {
		if sizingSet.IsTerminate {
			return
		}

		frame := float32(iFrame)

		{
			bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingLeftLegBone().Name()).Get(frame)
			bf.Rotation = leftLegRotations[i]
			sizingProcessMotion.InsertBoneFrame(sizingSet.SizingLeftLegBone().Name(), bf)
		}
		{
			bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingLeftKneeBone().Name()).Get(frame)
			bf.Rotation = leftKneeRotations[i]
			sizingProcessMotion.InsertBoneFrame(sizingSet.SizingLeftKneeBone().Name(), bf)
		}
		{
			bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingLeftAnkleBone().Name()).Get(frame)
			bf.Rotation = leftAnkleRotations[i]
			sizingProcessMotion.InsertBoneFrame(sizingSet.SizingLeftAnkleBone().Name(), bf)
		}
		{
			bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingRightLegBone().Name()).Get(frame)
			bf.Rotation = rightLegRotations[i]
			sizingProcessMotion.InsertBoneFrame(sizingSet.SizingRightLegBone().Name(), bf)
		}
		{
			bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingRightKneeBone().Name()).Get(frame)
			bf.Rotation = rightKneeRotations[i]
			sizingProcessMotion.InsertBoneFrame(sizingSet.SizingRightKneeBone().Name(), bf)
		}
		{
			bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingRightAnkleBone().Name()).Get(frame)
			bf.Rotation = rightAnkleRotations[i]
			sizingProcessMotion.InsertBoneFrame(sizingSet.SizingRightAnkleBone().Name(), bf)
		}

		if i > 0 && i%1000 == 0 {
			processLog("足補正10", sizingSet.Index, i, len(allFrames))
		}
	}

	if mlog.IsDebug() {
		outputVerboseMotion("足09", sizingSet.OutputMotionPath, sizingProcessMotion)
	}
}

func calcLegIkPositionY(
	index int, originalAnkleBone *pmx.Bone, originalToeTailDDelta, originalHeelDDelta,
	sizingToeTailDDelta, sizingHeelDDelta *delta.BoneDelta, moveScale *mmath.MVec3,
	toeBeforePositions, toeIdealPositions, heelBeforePositions, heelIdealPositions []*mmath.MVec3,
) float64 {
	if originalToeTailDDelta.FilledGlobalPosition().Y <= originalHeelDDelta.FilledGlobalPosition().Y ||
		mmath.NearEquals(originalToeTailDDelta.FilledGlobalPosition().Y, originalHeelDDelta.FilledGlobalPosition().Y, 1e-1) {
		// つま先の方がかかとより低い場合
		originalToeTailDY := originalToeTailDDelta.FilledGlobalPosition().Y

		// つま先のY座標を元モデルのつま先のY座標*スケールに合わせる
		idealSizingToeTailY := originalToeTailDY * moveScale.Y

		// 現時点のつま先のY座標(足IKの回転結果を適用させて求め直す)
		actualToeTailY := sizingToeTailDDelta.FilledGlobalPosition().Y

		if mlog.IsDebug() {
			toeBeforePositions[index] = sizingToeTailDDelta.FilledGlobalPosition().Copy()
			toeIdealPositions[index] = sizingToeTailDDelta.FilledGlobalPosition().Copy()
			toeIdealPositions[index].Y = idealSizingToeTailY
		}

		toeDiff := idealSizingToeTailY - actualToeTailY
		// toeDiff += originalAnkleBone.Position.Y - sizingAnkleBone.Position.Y
		lerpToeDiff := mmath.Lerp(toeDiff, 0,
			max(0, originalToeTailDDelta.FilledGlobalPosition().Y/originalAnkleBone.Position.Y))
		// 足首Y位置に近付くにつれて補正を弱める
		return lerpToeDiff
		// ankleDiff := sizingLeftToeTailY - originalLeftToeTailY
		// legIkPositions[index].Y += ankleDiff
	}

	// かかとの方がつま先より低い場合
	originalHeelDY := originalHeelDDelta.FilledGlobalPosition().Y

	// かかとのY座標を元モデルのかかとのY座標*スケールに合わせる
	idealSizingHeelY := originalHeelDY * moveScale.Y

	// 現時点のかかとのY座標
	actualSizingHeelY := sizingHeelDDelta.FilledGlobalPosition().Y

	if mlog.IsDebug() {
		heelBeforePositions[index] = sizingHeelDDelta.FilledGlobalPosition().Copy()
		heelIdealPositions[index] = sizingHeelDDelta.FilledGlobalPosition().Copy()
		heelIdealPositions[index].Y = idealSizingHeelY
	}

	heelDiff := idealSizingHeelY - actualSizingHeelY
	// heelDiff += originalAnkleBone.Position.Y - sizingAnkleBone.Position.Y
	lerpHeelDiff := mmath.Lerp(heelDiff, 0,
		originalHeelDDelta.FilledGlobalPosition().Y/originalAnkleBone.Position.Y)
	// 足首Y位置に近付くにつれて補正を弱める
	return lerpHeelDiff

	// ankleDiff := (sizingLeftHeelY - sizingLeftAnkleY) - (originalLeftHeelY - originalLeftAnkleY)
	// legIkPositions[index].Y += ankleDiff
}

func updateLegResultMotion(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, sizingProcessMotion *vmd.VmdMotion,
	incrementCompletedCount func(),
) error {
	// 足補正処理の結果をサイジング先モーションに反映
	sizingModel := sizingSet.SizingConfigModel
	outputMotion := sizingSet.OutputMotion

	activeFrames := getFrames(outputMotion, all_lower_leg_bone_names)

	// TOE_IK キーフレームのリセット
	outputMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.TOE_IK.Left()))
	outputMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.TOE_IK.Right()))

	// 足系はあるボーンだけ上書きする
	for _, boneName := range []string{
		pmx.CENTER.String(), pmx.GROOVE.String(), pmx.LOWER.String(),
		pmx.LEG_IK.Left(), pmx.LEG.Left(), pmx.KNEE.Left(), pmx.ANKLE.Left(),
		pmx.LEG_IK.Right(), pmx.LEG.Right(), pmx.KNEE.Right(), pmx.ANKLE.Right(),
	} {
		if !outputMotion.BoneFrames.Contains(boneName) {
			continue
		}
		outputMotion.BoneFrames.Get(boneName).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
			processBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
			bf.Position = processBf.FilledPosition().Copy()
			bf.Rotation = processBf.FilledRotation().Copy()
			outputMotion.BoneFrames.Get(boneName).Update(bf)
			return true
		})
	}

	// 中間キーフレのズレをチェック
	kneeThreshold := 0.3
	ankleThreshold := 0.2
	toeThreshold := 0.1

	err := miter.IterParallelByList(directions, 1, 1,
		func(dIndex int, direction pmx.BoneDirection) error {
			for tIndex, targetFrames := range [][]int{activeFrames, allFrames} {
				processAllDeltas, err := computeVmdDeltas(targetFrames, blockSize,
					sizingModel, sizingProcessMotion, sizingSet, true, all_lower_leg_bone_names, "足補正01")
				if err != nil {
					return err
				}

				for fIndex, iFrame := range targetFrames {
					if sizingSet.IsTerminate {
						return merr.TerminateError
					}
					frame := float32(iFrame)

					// 現時点の結果
					resultAllVmdDeltas, err := computeVmdDeltas([]int{iFrame}, 1,
						sizingModel, outputMotion, sizingSet, true, leg_direction_bone_names[dIndex], "")
					if err != nil {
						return err
					}

					// ひざの位置をチェック
					resultKneeDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.KNEE.StringFromDirection(direction))
					processKneeDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.KNEE.StringFromDirection(direction))

					// 足首の位置をチェック
					resultAnkleDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))
					processAnkleDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))

					// つま先親の位置をチェック
					resultToePDDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.TOE_P_D.StringFromDirection(direction))
					processToePDDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.TOE_P_D.StringFromDirection(direction))

					// 各関節位置がズレている場合、元の回転を焼き込む
					if resultKneeDelta.FilledGlobalPosition().Distance(processKneeDelta.FilledGlobalPosition()) > kneeThreshold ||
						resultAnkleDelta.FilledGlobalPosition().Distance(processAnkleDelta.FilledGlobalPosition()) > ankleThreshold ||
						resultToePDDelta.FilledGlobalPosition().Distance(processToePDDelta.FilledGlobalPosition()) > toeThreshold {
						for _, legBoneName := range []pmx.StandardBoneName{pmx.LEG, pmx.KNEE, pmx.ANKLE, pmx.LEG_IK} {
							boneName := legBoneName.StringFromDirection(direction)
							processBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
							resultBf := outputMotion.BoneFrames.Get(boneName).Get(frame)
							resultBf.Position = processBf.FilledPosition().Copy()
							resultBf.Rotation = processBf.FilledRotation().Copy()
							outputMotion.InsertBoneFrame(boneName, resultBf)
						}
					}

					if fIndex > 0 && fIndex%1000 == 0 {
						mlog.I(mi18n.T("足補正11", map[string]interface{}{
							"No":          sizingSet.Index + 1,
							"IterIndex":   fmt.Sprintf("%04d", iFrame),
							"AllCount":    fmt.Sprintf("%02d", len(targetFrames)),
							"Direction":   direction.String(),
							"FramesIndex": tIndex + 1}))
					}
				}
			}

			incrementCompletedCount()

			return nil
		}, nil)
	if err != nil {
		return err
	}

	if mlog.IsDebug() {
		outputVerboseMotion("足10", sizingSet.OutputMotionPath, outputMotion)
	}

	return err
}

func checkBonesForSizingLeg(sizingSet *domain.SizingSet) (err error) {

	for _, v := range [][]interface{}{
		{sizingSet.OriginalCenterBone, pmx.CENTER.String(), true},
		{sizingSet.OriginalLowerBone, pmx.LOWER.String(), true},
		{sizingSet.OriginalLegCenterBone, pmx.LEG_CENTER.String(), false},
		{sizingSet.OriginalTrunkRootBone, pmx.TRUNK_ROOT.String(), false},
		{sizingSet.OriginalLeftLegIkBone, pmx.LEG_IK.Left(), true},
		{sizingSet.OriginalLeftLegBone, pmx.LEG.Left(), true},
		{sizingSet.OriginalLeftKneeBone, pmx.KNEE.Left(), true},
		{sizingSet.OriginalLeftAnkleBone, pmx.ANKLE.Left(), true},
		{sizingSet.OriginalLeftToeIkBone, pmx.TOE_IK.Left(), true},
		{sizingSet.OriginalLeftToeTailBone, pmx.TOE_T.Left(), false},
		{sizingSet.OriginalLeftToeTailDBone, pmx.TOE_T_D.Left(), false},
		{sizingSet.OriginalLeftHeelBone, pmx.HEEL.Left(), false},
		{sizingSet.OriginalLeftHeelDBone, pmx.HEEL_D.Left(), false},
		{sizingSet.OriginalLeftToePBone, pmx.TOE_P.Left(), false},
		{sizingSet.OriginalLeftToePDBone, pmx.TOE_P_D.Left(), false},
		{sizingSet.OriginalRightLegIkBone, pmx.LEG_IK.Right(), true},
		{sizingSet.OriginalRightLegBone, pmx.LEG.Right(), true},
		{sizingSet.OriginalRightKneeBone, pmx.KNEE.Right(), true},
		{sizingSet.OriginalRightAnkleBone, pmx.ANKLE.Right(), true},
		{sizingSet.OriginalRightToeIkBone, pmx.TOE_IK.Right(), true},
		{sizingSet.OriginalRightToeTailBone, pmx.TOE_T.Right(), false},
		{sizingSet.OriginalRightToeTailDBone, pmx.TOE_T_D.Right(), false},
		{sizingSet.OriginalRightHeelBone, pmx.HEEL.Right(), false},
		{sizingSet.OriginalRightHeelDBone, pmx.HEEL_D.Right(), false},
		{sizingSet.OriginalRightToePBone, pmx.TOE_P.Right(), false},
		{sizingSet.OriginalRightToePDBone, pmx.TOE_P_D.Right(), false},
	} {
		getFunc := v[0].(func() *pmx.Bone)
		boneName := v[1].(string)
		isStandard := v[2].(bool)

		if getFunc() == nil {
			keyName := "ボーン不足エラー"
			if !isStandard {
				keyName = "検証ボーン不足エラー"
			}
			mlog.WT(mi18n.T("ボーン不足"), mi18n.T(keyName, map[string]interface{}{
				"Process": mi18n.T("足補正"), "No": sizingSet.Index + 1, "ModelType": "元モデル", "BoneName": boneName}))
			err = merr.NameNotFoundError
		}
	}

	// ------------------------------------------

	// グルーブはサイジング先に元々存在している場合のみ取得
	sizingSet.SizingGrooveVanillaBone()

	for _, v := range [][]interface{}{
		{sizingSet.SizingCenterBone, pmx.CENTER.String(), true},
		{sizingSet.SizingLowerBone, pmx.LOWER.String(), true},
		{sizingSet.SizingLegCenterBone, pmx.LEG_CENTER.String(), false},
		{sizingSet.SizingTrunkRootBone, pmx.TRUNK_ROOT.String(), false},
		{sizingSet.SizingLeftLegIkBone, pmx.LEG_IK.Left(), true},
		{sizingSet.SizingLeftLegBone, pmx.LEG.Left(), true},
		{sizingSet.SizingLeftKneeBone, pmx.KNEE.Left(), true},
		{sizingSet.SizingLeftAnkleBone, pmx.ANKLE.Left(), true},
		{sizingSet.SizingLeftToeIkBone, pmx.TOE_IK.Left(), true},
		{sizingSet.SizingLeftToeTailBone, pmx.TOE_T.Left(), false},
		{sizingSet.SizingLeftToeTailDBone, pmx.TOE_T_D.Left(), false},
		{sizingSet.SizingLeftHeelBone, pmx.HEEL.Left(), false},
		{sizingSet.SizingLeftHeelDBone, pmx.HEEL_D.Left(), false},
		{sizingSet.SizingLeftToePBone, pmx.TOE_P.Left(), false},
		{sizingSet.SizingLeftToePDBone, pmx.TOE_P_D.Left(), false},
		{sizingSet.SizingRightLegIkBone, pmx.LEG_IK.Right(), true},
		{sizingSet.SizingRightLegBone, pmx.LEG.Right(), true},
		{sizingSet.SizingRightKneeBone, pmx.KNEE.Right(), true},
		{sizingSet.SizingRightAnkleBone, pmx.ANKLE.Right(), true},
		{sizingSet.SizingRightToeIkBone, pmx.TOE_IK.Right(), true},
		{sizingSet.SizingRightToeTailBone, pmx.TOE_T.Right(), false},
		{sizingSet.SizingRightToeTailDBone, pmx.TOE_T_D.Right(), false},
		{sizingSet.SizingRightHeelBone, pmx.HEEL.Right(), false},
		{sizingSet.SizingRightHeelDBone, pmx.HEEL_D.Right(), false},
		{sizingSet.SizingRightToePBone, pmx.TOE_P.Right(), false},
		{sizingSet.SizingRightToePDBone, pmx.TOE_P_D.Right(), false},
	} {
		getFunc := v[0].(func() *pmx.Bone)
		boneName := v[1].(string)
		isStandard := v[2].(bool)

		if getFunc() == nil {
			keyName := "ボーン不足エラー"
			if !isStandard {
				keyName = "検証ボーン不足エラー"
			}
			mlog.WT(mi18n.T("ボーン不足"), mi18n.T(keyName, map[string]interface{}{
				"Process": mi18n.T("足補正"), "No": sizingSet.Index + 1, "ModelType": "先モデル", "BoneName": boneName}))
			err = merr.NameNotFoundError
		}
	}

	return err
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
