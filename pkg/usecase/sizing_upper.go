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

func SizingUpper(
	sizingSet *domain.SizingSet, moveScale *mmath.MVec3, sizingSetCount int, incrementCompletedCount func(),
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
	if checkBonesForSizingUpper(sizingSet) != nil {
		return false, err
	}

	// 処理は全フレームで行う
	allFrames := mmath.IntRanges(int(originalMotion.MaxFrame()) + 1)
	blockSize, _ := miter.GetBlockSize(len(allFrames) * sizingSetCount)

	// 元モデルのデフォーム結果を並列処理で取得
	originalAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.OriginalConfigModel, originalMotion, sizingSet, true, trunk_upper_bone_names, "上半身補正01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	sizingAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel, sizingProcessMotion, sizingSet, true, trunk_upper_bone_names, "上半身補正01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	var upperDistance float64
	upperDistance, err = calculateUpperDistance(sizingSet)
	if err != nil {
		return false, err
	}

	// サイジング先の上半身回転情報を取得(全フレーム処理してズレ検知用)
	var upperRotations, upper2Rotations []*mmath.MQuaternion
	upperRotations, upper2Rotations, err = calculateAdjustedUpper(
		sizingSet, allFrames, blockSize, sizingAllDeltas, originalAllDeltas, sizingProcessMotion, upperDistance)
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	updateUpper(sizingSet, allFrames, sizingProcessMotion, upperRotations, upper2Rotations)

	incrementCompletedCount()

	// sizingSet.OutputMotion = sizingProcessMotion
	// 足補正処理の結果をサイジング先モーションに反映
	if err = updateUpperResultMotion(sizingSet, allFrames, blockSize, sizingProcessMotion, incrementCompletedCount); err != nil {
		return false, err
	}

	sizingSet.CompletedSizingUpper = true

	return true, nil
}

func updateUpperResultMotion(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, sizingProcessMotion *vmd.VmdMotion, incrementCompletedCount func(),
) error {
	// 足補正処理の結果をサイジング先モーションに反映
	outputMotion := sizingSet.OutputMotion

	activeFrames := getFrames(outputMotion, trunk_upper_bone_names)

	// 先に差分を腕ボーンに適用
	for _, armBoneName := range []string{pmx.ARM.Left(), pmx.ARM.Right()} {
		outputMotion.BoneFrames.Get(armBoneName).ForEach(func(frame float32, bf *vmd.BoneFrame) {
			processUpperBf := sizingProcessMotion.BoneFrames.Get(pmx.UPPER.String()).Get(frame)
			outputUpperBf := outputMotion.BoneFrames.Get(pmx.UPPER.String()).Get(frame)

			diffQuat := outputUpperBf.Rotation.Muled(processUpperBf.Rotation.Inverted())

			if sizingSet.SizingUpper2VanillaBone() != nil {
				processUpper2Bf := sizingProcessMotion.BoneFrames.Get(pmx.UPPER2.String()).Get(frame)
				outputUpper2Bf := outputMotion.BoneFrames.Get(pmx.UPPER2.String()).Get(frame)

				diffQuat.Mul(outputUpper2Bf.Rotation.Muled(processUpper2Bf.Rotation.Inverted()))
			}

			armBf := outputMotion.BoneFrames.Get(armBoneName).Get(frame)
			armBf.Rotation = armBf.Rotation.Muled(diffQuat)
			outputMotion.BoneFrames.Get(armBoneName).Update(armBf)
		})
	}

	// 上半身はあるボーンだけ上書きする
	for _, upperBoneName := range []string{pmx.UPPER.String(), pmx.UPPER2.String()} {
		if !sizingSet.SizingModel.Bones.ContainsByName(upperBoneName) {
			continue
		}

		outputMotion.BoneFrames.Get(upperBoneName).ForEach(func(frame float32, bf *vmd.BoneFrame) {
			processBf := sizingProcessMotion.BoneFrames.Get(upperBoneName).Get(frame)
			bf.Rotation = processBf.Rotation.Copy()
			outputMotion.BoneFrames.Get(upperBoneName).Update(bf)
		})
	}

	// 中間キーフレのズレをチェック
	neckRootThreshold := 0.2

	for tIndex, targetFrames := range [][]int{activeFrames, allFrames} {
		processAllDeltas, err := computeVmdDeltas(targetFrames, blockSize,
			sizingSet.SizingModel, sizingProcessMotion, sizingSet, true, trunk_upper_bone_names, "上半身補正01")

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
				sizingSet.SizingModel, outputMotion, sizingSet, true, trunk_upper_bone_names, "")
			if err != nil {
				return err
			}

			// 首根元の位置をチェック
			resultNeckRootDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.NECK_ROOT.String())
			processNeckRootDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.NECK_ROOT.String())

			if resultNeckRootDelta.FilledGlobalPosition().Distance(processNeckRootDelta.FilledGlobalPosition()) > neckRootThreshold {
				// 各関節位置がズレている場合、元の回転を焼き込む
				for _, upperBone := range []*pmx.Bone{
					sizingSet.SizingUpperVanillaBone(), sizingSet.SizingUpper2VanillaBone(),
				} {
					if upperBone == nil {
						continue
					}
					processBf := sizingProcessMotion.BoneFrames.Get(upperBone.Name()).Get(frame)
					resultBf := outputMotion.BoneFrames.Get(upperBone.Name()).Get(frame)
					resultBf.Rotation = processBf.Rotation.Copy()
					outputMotion.InsertRegisteredBoneFrame(upperBone.Name(), resultBf)
				}
			}

			if fIndex > 0 && fIndex%1000 == 0 {
				mlog.I(mi18n.T("上半身補正04", map[string]interface{}{
					"No":          sizingSet.Index + 1,
					"IterIndex":   fmt.Sprintf("%04d", iFrame),
					"AllCount":    fmt.Sprintf("%02d", len(targetFrames)),
					"FramesIndex": tIndex + 1}))
			}
		}

		incrementCompletedCount()
	}

	return nil
}

func updateUpper(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	upperRotations, upper2Rotations []*mmath.MQuaternion,
) {
	for i, iFrame := range allFrames {
		frame := float32(iFrame)
		upperBf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingUpperBone().Name()).Get(frame)
		upperBf.Rotation = upperRotations[i]
		sizingProcessMotion.InsertRegisteredBoneFrame(sizingSet.SizingUpperBone().Name(), upperBf)

		if sizingSet.SizingUpper2VanillaBone() != nil {
			upper2Bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingUpper2Bone().Name()).Get(frame)
			upper2Bf.Rotation = upper2Rotations[i]
			sizingProcessMotion.InsertRegisteredBoneFrame(sizingSet.SizingUpper2Bone().Name(), upper2Bf)
		}

		if i > 0 && i%1000 == 0 {
			processLog("上半身補正03", sizingSet.Index, i, len(allFrames))
		}
	}
}

func calculateUpperDistanceRange(bones *pmx.Bones, upperRootIndex, neckRootIndex int) float64 {
	// distance := 0.0

	// startIndex := neckRootIndex

	// for range 100 {
	// 	bone, err := bones.Get(startIndex)
	// 	if err != nil || bone.ParentIndex == -1 {
	// 		break
	// 	}

	// 	parentBone, err := bones.Get(bone.ParentIndex)
	// 	if err != nil {
	// 		break
	// 	}

	// 	distance += bone.Position.Distance(parentBone.Position)
	// 	if parentBone.Index() == upperRootIndex {
	// 		break
	// 	}

	// 	startIndex = parentBone.Index()
	// }

	upperRootBone, err := bones.Get(upperRootIndex)
	if err != nil {
		return 0
	}

	neckRootBone, err := bones.Get(neckRootIndex)
	if err != nil {
		return 0
	}

	return upperRootBone.Position.Distance(neckRootBone.Position)
}

func calculateUpperDistance(sizingSet *domain.SizingSet) (float64, error) {
	// 上半身のスケールを取得
	originalDistance := calculateUpperDistanceRange(sizingSet.OriginalConfigModel.Bones,
		sizingSet.OriginalUpperRootBone().Index(), sizingSet.OriginalNeckRootBone().Index())

	sizingDistance := calculateUpperDistanceRange(sizingSet.SizingConfigModel.Bones,
		sizingSet.SizingUpperRootBone().Index(), sizingSet.SizingNeckRootBone().Index())

	if originalDistance == 0 || sizingDistance == 0 {
		return 0, merr.NameNotFoundError
	}

	return sizingDistance / originalDistance, nil
}

func createUpperIkBone(sizingSet *domain.SizingSet) *pmx.Bone {
	upperBones := make([]*pmx.Bone, 0)
	if sizingSet.SizingUpper2VanillaBone() != nil {
		upperBones = append(upperBones, sizingSet.SizingUpper2Bone())
	}
	if sizingSet.SizingUpperVanillaBone() != nil {
		upperBones = append(upperBones, sizingSet.SizingUpperBone())
	}

	// 上半身IK
	upperIkBone := pmx.NewBoneByName(fmt.Sprintf("%s%sIk", pmx.MLIB_PREFIX, pmx.UPPER.String()))
	upperIkBone.Position = sizingSet.SizingNeckRootBone().Position.Copy()
	upperIkBone.Ik = pmx.NewIk()
	upperIkBone.Ik.BoneIndex = sizingSet.SizingNeckRootBone().Index()
	upperIkBone.Ik.LoopCount = 100
	upperIkBone.Ik.UnitRotation = &mmath.MVec3{X: 0.1, Y: 0.0, Z: 0.0}
	upperIkBone.Ik.Links = make([]*pmx.IkLink, len(upperBones))
	for i, bone := range upperBones {
		upperIkBone.Ik.Links[i] = pmx.NewIkLink()
		upperIkBone.Ik.Links[i].BoneIndex = bone.Index()
	}

	return upperIkBone
}

func calculateAdjustedUpper(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int,
	sizingAllDeltas, originalAllDeltas []*delta.VmdDeltas, sizingProcessMotion *vmd.VmdMotion, upperDistance float64,
) (upperRotations, upper2Rotations []*mmath.MQuaternion, err error) {
	upperRotations = make([]*mmath.MQuaternion, len(allFrames))
	upper2Rotations = make([]*mmath.MQuaternion, len(allFrames))

	upperIkVanillaBone := createUpperIkBone(sizingSet)

	err = miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			originalUpperRootDelta := originalAllDeltas[index].Bones.GetByName(pmx.UPPER_ROOT.String())
			originalNeckRootDelta := originalAllDeltas[index].Bones.GetByName(pmx.NECK_ROOT.String())
			originalLocalUpperPosition := originalUpperRootDelta.FilledGlobalMatrix().Inverted().MulVec3(
				originalNeckRootDelta.FilledGlobalPosition())
			// originalLocalUpperPosition := originalNeckRootDelta.FilledGlobalPosition().Subed(
			// 	originalUpperRootDelta.FilledGlobalPosition())
			sizingLocalUpper := originalLocalUpperPosition.MuledScalar(upperDistance)

			sizingUpperRootDelta := sizingAllDeltas[index].Bones.GetByName(pmx.UPPER_ROOT.String())
			// sizingGlobalUpperPosition := sizingUpperRootDelta.FilledGlobalPosition().Added(sizingLocalUpper)
			sizingGlobalUpperPosition := sizingUpperRootDelta.FilledGlobalMatrix().MulVec3(sizingLocalUpper)

			sizingUpperDeltas := deform.DeformIk(sizingSet.SizingConfigModel, sizingProcessMotion, sizingAllDeltas[index], float32(data), upperIkVanillaBone, sizingGlobalUpperPosition, trunk_upper_bone_names, false)

			upperRotations[index] = sizingUpperDeltas.Bones.GetByName(pmx.UPPER.String()).FilledFrameRotation()
			if sizingSet.SizingUpper2VanillaBone() != nil {
				upper2Rotations[index] = sizingUpperDeltas.Bones.GetByName(pmx.UPPER2.String()).FilledFrameRotation()
			}

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("上半身補正02", sizingSet.Index, iterIndex, allCount)
		})

	return
}

func checkBonesForSizingUpper(sizingSet *domain.SizingSet) (err error) {

	for _, v := range [][]interface{}{
		{sizingSet.OriginalCenterBone, pmx.CENTER.String()},
		{sizingSet.OriginalTrunkRootBone, pmx.UPPER_ROOT.String()},
		{sizingSet.OriginalUpperBone, pmx.UPPER.String()},
		{sizingSet.OriginalNeckRootBone, pmx.NECK_ROOT.String()},
	} {
		getFunc := v[0].(func() *pmx.Bone)
		boneName := v[1].(string)

		if getFunc() == nil {
			mlog.WT(mi18n.T("ボーン不足"), mi18n.T("ボーン不足エラー", map[string]interface{}{
				"Process": mi18n.T("足補正"), "No": sizingSet.Index + 1, "ModelType": "元モデル", "BoneName": boneName}))
			err = merr.NameNotFoundError
		}
	}

	// ------------------------------------------

	// グルーブはサイジング先に元々存在している場合のみ取得
	sizingSet.SizingGrooveVanillaBone()
	sizingSet.SizingUpper2VanillaBone()

	for _, v := range [][]interface{}{
		{sizingSet.SizingCenterBone, pmx.CENTER.String()},
		{sizingSet.SizingTrunkRootBone, pmx.UPPER_ROOT.String()},
		{sizingSet.SizingUpperBone, pmx.UPPER.String()},
		{sizingSet.SizingNeckRootBone, pmx.NECK_ROOT.String()},
	} {
		getFunc := v[0].(func() *pmx.Bone)
		boneName := v[1].(string)

		if getFunc() == nil {
			mlog.WT(mi18n.T("ボーン不足"), mi18n.T("ボーン不足エラー", map[string]interface{}{
				"Process": mi18n.T("足補正"), "No": sizingSet.Index + 1, "ModelType": "先モデル", "BoneName": boneName}))
			err = merr.NameNotFoundError
		}
	}

	return err
}
