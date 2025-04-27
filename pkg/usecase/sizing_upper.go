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

type SizingUpperUsecase struct {
}

func NewSizingUpperUsecase() *SizingUpperUsecase {
	return &SizingUpperUsecase{}
}

func (su *SizingUpperUsecase) Exec(
	sizingSet *domain.SizingSet, sizingSetCount int, incrementCompletedCount func(),
) (bool, error) {
	// 対象外の場合は何もせず終了
	if !sizingSet.IsSizingUpper || sizingSet.CompletedSizingUpper {
		return false, nil
	}

	originalMotion := sizingSet.OriginalMotion
	sizingProcessMotion, err := sizingSet.OutputMotion.Copy()
	if err != nil {
		return false, err
	}

	mlog.I(mi18n.T("上半身補正開始", map[string]interface{}{"No": sizingSet.Index + 1}))

	// 処理対象ボーンチェック
	if err := su.checkBones(sizingSet); err != nil {
		return false, err
	}

	// 処理は全フレームで行う
	allFrames := mmath.IntRanges(int(originalMotion.MaxFrame()) + 1)
	blockSize, _ := miter.GetBlockSize(len(allFrames) * sizingSetCount)

	// [焼き込み] -----------------------

	// 元モデルのデフォーム結果を並列処理で取得
	originalAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.OriginalConfigModel,
		originalMotion, sizingSet, true, trunk_upper_bone_names, "上半身補正01", incrementCompletedCount)
	if err != nil {
		return false, err
	}

	// 元モデルのモーフデフォーム結果を並列処理で取得
	originalMorphAllDeltas, err := computeMorphVmdDeltas(allFrames, blockSize, sizingSet.OriginalConfigModel,
		originalMotion, sizingSet, trunk_upper_bone_names, "上半身補正01", incrementCompletedCount)
	if err != nil {
		return false, err
	}

	// 先モデルのモーフデフォーム結果を並列処理で取得
	sizingMorphAllDeltas, err := computeMorphVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel,
		originalMotion, sizingSet, trunk_upper_bone_names, "上半身補正01", incrementCompletedCount)
	if err != nil {
		return false, err
	}

	// [上半身] -----------------------

	{
		if mlog.IsDebug() {
			outputVerboseMotion("上01", sizingSet.OutputMotionPath, sizingProcessMotion)
		}

		// 先モデルの足中心までのデフォーム結果を並列処理で取得
		sizingAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel,
			sizingProcessMotion, sizingSet, true, trunk_upper_bone_names, "上半身補正01", incrementCompletedCount)
		if err != nil {
			return false, err
		}

		// サイジング先の上半身回転情報を取得(全フレーム処理してズレ検知用)
		upperRotations, upper2Rotations, neckRotations, leftArmRotations, rightArmRotations, err :=
			su.calculateAdjustedUpper(
				sizingSet, allFrames, blockSize,
				originalAllDeltas, sizingAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas,
				sizingProcessMotion, incrementCompletedCount, "上02")
		if err != nil {
			return false, err
		}

		su.updateUpper(sizingSet, allFrames, sizingProcessMotion, upperRotations, upper2Rotations,
			neckRotations, leftArmRotations, rightArmRotations, incrementCompletedCount)

		if mlog.IsDebug() {
			outputVerboseMotion("上03", sizingSet.OutputMotionPath, sizingProcessMotion)
		}
	}

	{
		if err = su.updateOutputMotion(
			sizingSet, allFrames, blockSize, sizingProcessMotion, "上04",
			incrementCompletedCount,
		); err != nil {
			return false, err
		}
	}

	sizingSet.CompletedSizingUpper = true

	return true, nil
}

// func (su *SizingUpperUsecase) updateUpper(
// 	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
// 	upperRotations, upper2Rotations, neckRotations, leftArmRotations, rightArmRotations []*mmath.MQuaternion,
// ) {
// 	for i, iFrame := range allFrames {
// 		frame := float32(iFrame)
// 		upperBf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingUpperBone().Name()).Get(frame)
// 		upperBf.Rotation = upperRotations[i]
// 		sizingProcessMotion.InsertBoneFrame(sizingSet.SizingUpperBone().Name(), upperBf)

// 		if sizingSet.SizingUpper2VanillaBone() != nil {
// 			upper2Bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingUpper2Bone().Name()).Get(frame)
// 			upper2Bf.Rotation = upper2Rotations[i]
// 			sizingProcessMotion.InsertBoneFrame(sizingSet.SizingUpper2Bone().Name(), upper2Bf)
// 		}

// 		{
// 			neckBf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingNeckRootBone().Name()).Get(frame)
// 			if upper2CancelRotations != nil && upper2CancelRotations[i] != nil {
// 				neckBf.Rotation = upper2CancelRotations[i].Muled(upperCancelRotations[i]).Muled(neckBf.FilledRotation())
// 			} else {
// 				neckBf.Rotation = upperCancelRotations[i].Muled(neckBf.FilledRotation())
// 			}
// 			sizingProcessMotion.InsertBoneFrame(sizingSet.SizingNeckRootBone().Name(), neckBf)
// 		}
// 		{
// 			for _, direction := range directions {
// 				armBf := sizingProcessMotion.BoneFrames.Get(pmx.ARM.StringFromDirection(direction)).Get(frame)
// 				if upper2CancelRotations != nil && upper2CancelRotations[i] != nil {
// 					armBf.Rotation = upper2CancelRotations[i].Muled(upperCancelRotations[i]).Muled(armBf.FilledRotation())
// 				} else {
// 					armBf.Rotation = upperCancelRotations[i].Muled(armBf.FilledRotation())
// 				}
// 				sizingProcessMotion.InsertBoneFrame(pmx.ARM.StringFromDirection(direction), armBf)
// 			}
// 		}

// 		if i > 0 && i%1000 == 0 {
// 			processLog("上半身補正03", sizingSet.Index, i, len(allFrames))
// 		}
// 	}

// 	if mlog.IsDebug() {
// 		outputVerboseMotion("上半身02", sizingSet.OutputMotionPath, sizingProcessMotion)
// 	}
// }

// func (su *SizingUpperUsecase) calculateUpperDistanceRange(bones *pmx.Bones, upperRootIndex, neckRootIndex int) float64 {
// 	distance := 0.0

// 	startIndex := neckRootIndex

// 	for range 100 {
// 		bone, err := bones.Get(startIndex)
// 		if err != nil || bone.ParentIndex == -1 {
// 			break
// 		}

// 		parentBone, err := bones.Get(bone.ParentIndex)
// 		if err != nil {
// 			break
// 		}

// 		distance += bone.Position.Distance(parentBone.Position)
// 		if parentBone.Index() == upperRootIndex {
// 			break
// 		}

// 		startIndex = parentBone.Index()
// 	}

// 	return distance

// 	// upperRootBone, err := bones.Get(upperRootIndex)
// 	// if err != nil {
// 	// 	return 0
// 	// }

// 	// neckRootBone, err := bones.Get(neckRootIndex)
// 	// if err != nil {
// 	// 	return 0
// 	// }

// 	// return upperRootBone.Position.Distance(neckRootBone.Position)
// }

// func (su *SizingUpperUsecase) calculateUpperDistance(sizingSet *domain.SizingSet) (float64, error) {
// 	// 上半身のスケールを取得
// 	originalDistance := su.calculateUpperDistanceRange(sizingSet.OriginalConfigModel.Bones,
// 		sizingSet.OriginalUpperRootBone().Index(), sizingSet.OriginalNeckRootBone().Index())

// 	sizingDistance := su.calculateUpperDistanceRange(sizingSet.SizingConfigModel.Bones,
// 		sizingSet.SizingUpperRootBone().Index(), sizingSet.SizingNeckRootBone().Index())

// 	if originalDistance == 0 || sizingDistance == 0 {
// 		return 0, merr.NameNotFoundError
// 	}

// 	return sizingDistance / originalDistance, nil
// }

func (su *SizingUpperUsecase) createUpperIkBone(sizingSet *domain.SizingSet) *pmx.Bone {
	upperBone := sizingSet.SizingUpperBone()
	// upper2Bone := sizingSet.SizingUpper2Bone()
	ikTargetBone := sizingSet.SizingNeckRootBone()

	// 上半身IK
	ikBone := pmx.NewBoneByName(fmt.Sprintf("%s%sIk", pmx.MLIB_PREFIX, upperBone.Name()))
	ikBone.Position = ikTargetBone.Position.Copy()
	ikBone.Ik = pmx.NewIk()
	ikBone.Ik.BoneIndex = ikTargetBone.Index()
	ikBone.Ik.LoopCount = 10
	ikBone.Ik.UnitRotation = &mmath.MVec3{X: 1, Y: 0.0, Z: 0.0}
	ikBone.Ik.Links = make([]*pmx.IkLink, 0)
	for _, parentBoneIndex := range ikTargetBone.ParentBoneIndexes {
		link := pmx.NewIkLink()
		link.BoneIndex = parentBoneIndex
		if parentBoneIndex != upperBone.Index() {
			// 上半身以外は動かさない
			link.AngleLimit = true
		}
		ikBone.Ik.Links = append(ikBone.Ik.Links, link)

		if parentBoneIndex == upperBone.Index() {
			// 上半身までいったら終了
			break
		}
	}

	return ikBone
}

func (su *SizingUpperUsecase) calculateAdjustedUpper(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int,
	originalAllDeltas, sizingAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas []*delta.VmdDeltas,
	sizingProcessMotion *vmd.VmdMotion, incrementCompletedCount func(), verboseMotionName string,
) (
	sizingUpperResultRotations, sizingUpper2ResultRotations, sizingNeckResultRotations,
	sizingLeftArmResultRotations, sizingRightArmResultRotations []*mmath.MQuaternion,
	err error,
) {
	sizingUpperResultRotations = make([]*mmath.MQuaternion, len(allFrames))
	sizingUpper2ResultRotations = make([]*mmath.MQuaternion, len(allFrames))
	sizingNeckResultRotations = make([]*mmath.MQuaternion, len(allFrames))
	sizingLeftArmResultRotations = make([]*mmath.MQuaternion, len(allFrames))
	sizingRightArmResultRotations = make([]*mmath.MQuaternion, len(allFrames))

	upperIkBone := su.createUpperIkBone(sizingSet)

	originalUpperInitialPositions := make([]*mmath.MVec3, len(allFrames))
	originalUpperInitialRotations := make([]*mmath.MQuaternion, len(allFrames))
	originalUpper2InitialPositions := make([]*mmath.MVec3, len(allFrames))
	originalUpper2InitialRotations := make([]*mmath.MQuaternion, len(allFrames))
	originalNeckRootInitialPositions := make([]*mmath.MVec3, len(allFrames))
	originalNeckInitialPositions := make([]*mmath.MVec3, len(allFrames))
	originalLeftArmInitialPositions := make([]*mmath.MVec3, len(allFrames))
	originalRightArmInitialPositions := make([]*mmath.MVec3, len(allFrames))
	originalNeckInitialRotations := make([]*mmath.MQuaternion, len(allFrames))
	originalLeftArmInitialRotations := make([]*mmath.MQuaternion, len(allFrames))
	originalRightArmInitialRotations := make([]*mmath.MQuaternion, len(allFrames))

	sizingNeckRootIdealPositions := make([]*mmath.MVec3, len(allFrames))

	sizingUpperInitialPositions := make([]*mmath.MVec3, len(allFrames))
	sizingUpperInitialRotations := make([]*mmath.MQuaternion, len(allFrames))
	sizingUpper2InitialPositions := make([]*mmath.MVec3, len(allFrames))
	sizingUpper2InitialRotations := make([]*mmath.MQuaternion, len(allFrames))
	sizingNeckRootInitialPositions := make([]*mmath.MVec3, len(allFrames))
	sizingNeckInitialPositions := make([]*mmath.MVec3, len(allFrames))
	sizingLeftArmInitialPositions := make([]*mmath.MVec3, len(allFrames))
	sizingRightArmInitialPositions := make([]*mmath.MVec3, len(allFrames))
	sizingNeckInitialRotations := make([]*mmath.MQuaternion, len(allFrames))
	sizingLeftArmInitialRotations := make([]*mmath.MQuaternion, len(allFrames))
	sizingRightArmInitialRotations := make([]*mmath.MQuaternion, len(allFrames))

	sizingUpperResultPositions := make([]*mmath.MVec3, len(allFrames))
	sizingUpper2ResultPositions := make([]*mmath.MVec3, len(allFrames))
	sizingNeckRootResultPositions := make([]*mmath.MVec3, len(allFrames))
	sizingNeckResultPositions := make([]*mmath.MVec3, len(allFrames))
	sizingLeftArmResultPositions := make([]*mmath.MVec3, len(allFrames))
	sizingRightArmResultPositions := make([]*mmath.MVec3, len(allFrames))

	err = miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			// 上半身から首根元の傾き
			originalMorphUpperDelta := originalMorphAllDeltas[index].Bones.GetByName(pmx.UPPER.String())
			originalMorphNeckRootDelta := originalMorphAllDeltas[index].Bones.GetByName(pmx.NECK_ROOT.String())
			sizingMorphUpperDelta := sizingMorphAllDeltas[index].Bones.GetByName(pmx.UPPER.String())
			sizingMorphNeckRootDelta := sizingMorphAllDeltas[index].Bones.GetByName(pmx.NECK_ROOT.String())

			// 上半身と首根元の相対位置
			originalMorphNeckRootLocalPosition := originalMorphUpperDelta.FilledGlobalMatrix().Inverted().MulVec3(
				originalMorphNeckRootDelta.FilledGlobalPosition())
			sizingMorphNeckRootLocalPosition := sizingMorphUpperDelta.FilledGlobalMatrix().Inverted().MulVec3(
				sizingMorphNeckRootDelta.FilledGlobalPosition())

			// 真上から首根元までの傾き
			originalUpperSlope := mmath.NewMQuaternionRotate(mmath.MVec3UnitY, originalMorphNeckRootLocalPosition.Normalized())
			sizingUpperSlope := mmath.NewMQuaternionRotate(mmath.MVec3UnitY, sizingMorphNeckRootLocalPosition.Normalized())

			// 上半身から首根元までを真っ直ぐにしたときの長さ差
			originalNeckRootVerticalDiff := originalUpperSlope.Inverted().MulVec3(originalMorphNeckRootLocalPosition).Truncate(1e-3)

			sizingNeckRootVerticalDiff := sizingUpperSlope.Inverted().MulVec3(sizingMorphNeckRootLocalPosition).Truncate(1e-3)

			neckRootFromUpperScale := sizingNeckRootVerticalDiff.Dived(originalNeckRootVerticalDiff).Effective().One()

			// -------------------------

			originalUpperRootDelta := originalAllDeltas[index].Bones.GetByName(pmx.UPPER_ROOT.String())
			originalUpperDelta := originalAllDeltas[index].Bones.GetByName(pmx.UPPER.String())
			originalNeckRootDelta := originalAllDeltas[index].Bones.GetByName(pmx.NECK_ROOT.String())

			// 上半身の軸回転を取得
			upperTwistQuat, _ := originalUpperDelta.FilledFrameRotation().SeparateTwistByAxis(mmath.MVec3UnitYNeg)
			upperTwistMat := upperTwistQuat.ToMat4()

			// 上半身根元に上半身の軸回転を加えたところから見た足ボーンのローカル位置
			originalNeckRootLocalPosition := originalUpperRootDelta.FilledGlobalMatrix().Copy().Muled(
				upperTwistMat).Inverted().MulVec3(originalNeckRootDelta.FilledGlobalPosition())

			// 真っ直ぐにしたときのローカル位置
			originalNeckRootVerticalLocalPosition := originalUpperSlope.Inverted().MulVec3(originalNeckRootLocalPosition).Truncate(1e-3)

			// スケール差を考慮した先の首根元ボーンのローカル位置
			sizingNeckRootVerticalLocalPosition := originalNeckRootVerticalLocalPosition.Muled(neckRootFromUpperScale)

			sizingNeckRootLocalPosition := sizingUpperSlope.MulVec3(sizingNeckRootVerticalLocalPosition)

			sizingUpperRootDelta := sizingAllDeltas[index].Bones.GetByName(pmx.UPPER_ROOT.String())
			sizingUpperDelta := sizingAllDeltas[index].Bones.GetByName(pmx.UPPER.String())
			sizingUpperInitialRotations[index] = sizingUpperDelta.FilledFrameRotation().Copy()

			sizingUpper2Delta := sizingAllDeltas[index].Bones.GetByName(pmx.UPPER2.String())
			if sizingUpper2Delta != nil {
				sizingUpper2InitialRotations[index] = sizingUpper2Delta.FilledFrameRotation().Copy()
			}

			sizingNeckRootIdealGlobalPosition := sizingUpperRootDelta.FilledGlobalMatrix().Muled(
				upperTwistMat).MulVec3(sizingNeckRootLocalPosition)

			if mlog.IsDebug() {
				originalNeckRootInitialPositions[index] = originalNeckRootDelta.FilledGlobalPosition().Copy()

				originalUpperDelta := originalAllDeltas[index].Bones.GetByName(pmx.UPPER.String())
				originalUpperInitialPositions[index] = originalUpperDelta.FilledGlobalPosition().Copy()
				originalUpperInitialRotations[index] = originalUpperDelta.FilledFrameRotation().Copy()

				originalUpper2Delta := originalAllDeltas[index].Bones.GetByName(pmx.UPPER2.String())
				if originalUpper2Delta != nil {
					originalUpper2InitialPositions[index] = originalUpper2Delta.FilledGlobalPosition().Copy()
					originalUpper2InitialRotations[index] = originalUpper2Delta.FilledFrameRotation().Copy()
				}

				originalLeftArmDelta := originalAllDeltas[index].Bones.GetByName(pmx.ARM.Left())
				originalLeftArmInitialPositions[index] = originalLeftArmDelta.FilledGlobalPosition().Copy()
				originalLeftArmInitialRotations[index] = originalLeftArmDelta.FilledFrameRotation().Copy()

				originalRightArmDelta := originalAllDeltas[index].Bones.GetByName(pmx.ARM.Right())
				originalRightArmInitialPositions[index] = originalRightArmDelta.FilledGlobalPosition().Copy()
				originalRightArmInitialRotations[index] = originalRightArmDelta.FilledFrameRotation().Copy()

				originalNeckDelta := originalAllDeltas[index].Bones.GetByName(pmx.NECK.String())
				originalNeckInitialPositions[index] = originalNeckDelta.FilledGlobalPosition().Copy()
				originalNeckInitialRotations[index] = originalNeckDelta.FilledFrameRotation().Copy()

				// ------

				sizingNeckRootInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(
					pmx.NECK_ROOT.String()).FilledGlobalPosition().Copy()

				sizingUpperInitialPositions[index] = sizingUpperDelta.FilledGlobalPosition().Copy()

				if sizingUpper2Delta != nil {
					sizingUpper2InitialPositions[index] = sizingUpper2Delta.FilledGlobalPosition().Copy()
				}

				sizingLeftArmDelta := sizingAllDeltas[index].Bones.GetByName(pmx.ARM.Left())
				sizingLeftArmInitialPositions[index] = sizingLeftArmDelta.FilledGlobalPosition().Copy()
				sizingLeftArmInitialRotations[index] = sizingLeftArmDelta.FilledFrameRotation().Copy()

				sizingRightArmDelta := sizingAllDeltas[index].Bones.GetByName(pmx.ARM.Right())
				sizingRightArmInitialPositions[index] = sizingRightArmDelta.FilledGlobalPosition().Copy()
				sizingRightArmInitialRotations[index] = sizingRightArmDelta.FilledFrameRotation().Copy()

				sizingNeckDelta := sizingAllDeltas[index].Bones.GetByName(pmx.NECK.String())
				sizingNeckInitialPositions[index] = sizingNeckDelta.FilledGlobalPosition().Copy()
				sizingNeckInitialRotations[index] = sizingNeckDelta.FilledFrameRotation().Copy()

				// ------

				sizingNeckRootIdealPositions[index] = sizingNeckRootIdealGlobalPosition.Copy()
			}

			// IK解決
			sizingUpperDeltas := deform.DeformIks(sizingSet.SizingConfigModel, sizingProcessMotion,
				sizingAllDeltas[index], float32(data),
				[]*pmx.Bone{upperIkBone},
				[]*pmx.Bone{sizingSet.SizingNeckRootBone()},
				[]*mmath.MVec3{sizingNeckRootIdealGlobalPosition},
				trunk_upper_bone_names, 1, false, false)

			sizingUpperResultDelta := sizingUpperDeltas.Bones.GetByName(pmx.UPPER.String())
			sizingUpperResultRotations[index] = sizingUpperResultDelta.FilledFrameRotation().Copy()

			upperDiffQuat := sizingUpperResultRotations[index].Muled(
				sizingUpperInitialRotations[index].Inverted())

			sizingUpper2ResultDelta := sizingUpperDeltas.Bones.GetByName(pmx.UPPER2.String())
			if sizingUpper2ResultDelta != nil && sizingUpper2InitialRotations[index] != nil {
				sizingUpper2ResultRotations[index] = sizingUpper2ResultDelta.FilledFrameRotation().Copy()
				upper2DiffQuat := sizingUpper2ResultRotations[index].Muled(
					sizingUpper2InitialRotations[index].Inverted())
				upperDiffQuat.Mul(upper2DiffQuat)
			}

			sizingNeckResultDelta := sizingUpperDeltas.Bones.GetByName(pmx.NECK.String())
			sizingNeckResultRotations[index] = upperDiffQuat.Inverted().Muled(sizingNeckResultDelta.FilledFrameRotation())

			sizingLeftArmResultDelta := sizingUpperDeltas.Bones.GetByName(pmx.ARM.Left())
			sizingLeftArmResultRotations[index] = upperDiffQuat.Inverted().Muled(sizingLeftArmResultDelta.FilledFrameRotation())

			sizingRightArmResultDelta := sizingUpperDeltas.Bones.GetByName(pmx.ARM.Right())
			sizingRightArmResultRotations[index] = upperDiffQuat.Inverted().Muled(sizingRightArmResultDelta.FilledFrameRotation())

			if mlog.IsDebug() {
				sizingUpperResultPositions[index] = sizingUpperResultDelta.FilledGlobalPosition().Copy()

				if sizingUpper2ResultDelta != nil {
					sizingUpper2ResultPositions[index] = sizingUpper2ResultDelta.FilledGlobalPosition().Copy()
				}

				sizingNeckRootResultDelta := sizingUpperDeltas.Bones.GetByName(pmx.NECK_ROOT.String())
				sizingNeckRootResultPositions[index] = sizingNeckRootResultDelta.FilledGlobalPosition().Copy()

				sizingNeckResultPositions[index] = sizingNeckResultDelta.FilledGlobalPosition().Copy()

				sizingLeftArmResultPositions[index] = sizingLeftArmResultDelta.FilledGlobalPosition().Copy()
				sizingRightArmResultPositions[index] = sizingRightArmResultDelta.FilledGlobalPosition().Copy()
			}

			incrementCompletedCount()

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("上半身補正02", sizingSet.Index, iterIndex, allCount)
		})
	if err != nil {
		return nil, nil, nil, nil, nil, err
	}

	if mlog.IsDebug() {
		motion := vmd.NewVmdMotion("")
		initialRotations := make([]*mmath.MQuaternion, len(allFrames))

		for i, iFrame := range allFrames {
			frame := float32(iFrame)

			for _, v := range [][]any{
				{originalUpperInitialPositions, originalUpperInitialRotations, "元今上半身"},
				{originalUpper2InitialPositions, originalUpper2InitialRotations, "元今上半身2"},
				{originalNeckRootInitialPositions, initialRotations, "元今首根元"},
				{originalNeckInitialPositions, originalNeckInitialRotations, "元今首"},
				{originalLeftArmInitialPositions, originalLeftArmInitialRotations, "元今左腕"},
				{originalRightArmInitialPositions, originalRightArmInitialRotations, "元今右腕"},
				{sizingUpperInitialPositions, sizingUpperInitialRotations, "先今上半身"},
				{sizingUpper2InitialPositions, sizingUpper2InitialRotations, "先今上半身2"},
				{sizingNeckRootInitialPositions, initialRotations, "先今首根元"},
				{sizingNeckInitialPositions, sizingNeckInitialRotations, "先今首"},
				{sizingLeftArmInitialPositions, sizingLeftArmInitialRotations, "先今左腕"},
				{sizingRightArmInitialPositions, sizingRightArmInitialRotations, "先今右腕"},
				{sizingNeckRootIdealPositions, initialRotations, "先理首根元"},
				{sizingUpperResultPositions, sizingUpperResultRotations, "先結上半身"},
				{sizingUpper2ResultPositions, sizingUpper2ResultRotations, "先結上半身2"},
				{sizingNeckRootResultPositions, initialRotations, "先結首根元"},
				{sizingNeckResultPositions, sizingNeckResultRotations, "先結首"},
				{sizingLeftArmResultPositions, sizingLeftArmResultRotations, "先結左腕"},
				{sizingRightArmResultPositions, sizingRightArmResultRotations, "先結右腕"},
			} {
				positions := v[0].([]*mmath.MVec3)
				rotations := v[1].([]*mmath.MQuaternion)
				boneName := v[2].(string)
				bf := vmd.NewBoneFrame(frame)
				bf.Position = positions[i]
				bf.Rotation = rotations[i]
				motion.InsertBoneFrame(boneName, bf)
			}
		}

		outputVerboseMotion(verboseMotionName, sizingSet.OutputMotionPath, motion)
	}

	return sizingUpperResultRotations, sizingUpper2ResultRotations, sizingNeckResultRotations,
		sizingLeftArmResultRotations, sizingRightArmResultRotations, nil
}

// updateLower は、補正した下半身回転をサイジング先モーションに反映します。
func (su *SizingUpperUsecase) updateUpper(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	sizingUpperResultRotations, sizingUpper2ResultRotations, sizingNeckResultRotations,
	sizingLeftArmResultRotations, sizingRightArmResultRotations []*mmath.MQuaternion, incrementCompletedCount func(),
) {
	for i, iFrame := range allFrames {
		frame := float32(iFrame)

		for _, v := range [][]any{
			{sizingUpperResultRotations, pmx.UPPER.String()},
			{sizingUpper2ResultRotations, pmx.UPPER2.String()},
			{sizingNeckResultRotations, pmx.NECK.String()},
			{sizingLeftArmResultRotations, pmx.ARM.Left()},
			{sizingRightArmResultRotations, pmx.ARM.Right()},
		} {
			rotations := v[0].([]*mmath.MQuaternion)
			boneName := v[1].(string)

			bf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
			bf.Rotation = rotations[i]
			sizingProcessMotion.InsertBoneFrame(boneName, bf)
		}

		if i > 0 && i%1000 == 0 {
			processLog("上半身補正03", sizingSet.Index, i, len(allFrames))
		}

		incrementCompletedCount()
	}
}

func (su *SizingUpperUsecase) updateOutputMotion(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, sizingProcessMotion *vmd.VmdMotion,
	verboseMotionKey string, incrementCompletedCount func(),
) error {
	// 補正の結果をサイジング先モーションに反映
	sizingModel := sizingSet.SizingConfigModel
	outputMotion := sizingSet.OutputMotion

	targetBoneNames := []string{
		pmx.UPPER.String(), pmx.UPPER2.String(), pmx.NECK.String(),
		pmx.ARM.Left(), pmx.ARM.Right(),
	}

	activeFrames := getFrames(outputMotion, targetBoneNames)
	intervalFrames := mmath.IntRangesByStep(allFrames[0], allFrames[len(allFrames)-1], 4)

	// まずはあるボーンだけ上書きする
	for _, boneName := range targetBoneNames {
		if !outputMotion.BoneFrames.Contains(boneName) {
			continue
		}
		outputMotion.BoneFrames.Get(boneName).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
			processBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
			bf.Rotation = processBf.FilledRotation().Copy()
			outputMotion.BoneFrames.Get(boneName).Update(bf)
			return true
		})
	}

	if mlog.IsDebug() {
		outputVerboseMotion(verboseMotionKey, sizingSet.OutputMotionPath, outputMotion)
	}

	// 中間キーフレのズレをチェック
	neckRootThreshold := 0.2

	for tIndex, targetFrames := range [][]int{activeFrames, intervalFrames, allFrames} {
		processAllDeltas, err := computeVmdDeltas(targetFrames, blockSize,
			sizingModel, sizingProcessMotion, sizingSet, true, trunk_upper_bone_names, "上半身補正01", incrementCompletedCount)
		if err != nil {
			return err
		}

		prevLog := 0
		prevFrame := 0
		for fIndex, iFrame := range targetFrames {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}
			frame := float32(iFrame)

			// 現時点の結果
			resultAllVmdDeltas, err := computeVmdDeltas([]int{iFrame}, 1,
				sizingModel, outputMotion, sizingSet, true, trunk_upper_bone_names, "", nil)
			if err != nil {
				return err
			}

			// 首根元の位置をチェック
			resultNeckRootDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.NECK_ROOT.String())
			processNeckRootDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.NECK_ROOT.String())

			if resultNeckRootDelta.FilledGlobalPosition().Distance(processNeckRootDelta.FilledGlobalPosition()) > neckRootThreshold {
				for _, boneName := range []string{pmx.UPPER.String(), pmx.UPPER2.String(), pmx.NECK.String(), pmx.ARM.Left(), pmx.ARM.Right()} {
					if !outputMotion.BoneFrames.Contains(boneName) {
						continue
					}

					processBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
					resultBf := outputMotion.BoneFrames.Get(boneName).Get(frame)
					resultBf.Rotation = processBf.FilledRotation().Copy()
					outputMotion.InsertBoneFrame(boneName, resultBf)
				}
			}

			if fIndex > 0 && int(iFrame/1000) > prevLog {
				mlog.I(mi18n.T("上半身補正04", map[string]interface{}{
					"No":          sizingSet.Index + 1,
					"IterIndex":   fmt.Sprintf("%04d", iFrame),
					"AllCount":    fmt.Sprintf("%04d", allFrames[len(allFrames)-1]),
					"FramesIndex": tIndex + 1}))
				prevLog = int(iFrame / 1000)
			}

			for f := prevFrame + 1; f <= iFrame; f++ {
				incrementCompletedCount()
			}

			prevFrame = iFrame
		}

		if mlog.IsDebug() {
			outputVerboseMotion(fmt.Sprintf("%s_%d", verboseMotionKey, tIndex),
				sizingSet.OutputMotionPath, outputMotion)
		}
	}

	return nil
}

func (su *SizingUpperUsecase) checkBones(sizingSet *domain.SizingSet) (err error) {

	for _, v := range [][]interface{}{
		{sizingSet.OriginalCenterBone, pmx.CENTER.String(), true},
		{sizingSet.OriginalTrunkRootBone, pmx.UPPER_ROOT.String(), false},
		{sizingSet.OriginalUpperBone, pmx.UPPER.String(), true},
		{sizingSet.OriginalNeckRootBone, pmx.NECK_ROOT.String(), false},
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
	sizingSet.SizingUpper2VanillaBone()

	for _, v := range [][]interface{}{
		{sizingSet.SizingCenterBone, pmx.CENTER.String(), true},
		{sizingSet.SizingTrunkRootBone, pmx.UPPER_ROOT.String(), false},
		{sizingSet.SizingUpperBone, pmx.UPPER.String(), true},
		{sizingSet.SizingNeckRootBone, pmx.NECK_ROOT.String(), false},
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
