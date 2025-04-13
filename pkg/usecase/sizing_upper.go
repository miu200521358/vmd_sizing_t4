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
	if err := checkBonesForSizingUpper(sizingSet); err != nil {
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

	// サイジング先の上半身回転情報を取得(全フレーム処理してズレ検知用)
	if err := calculateAdjustedUpper(
		sizingSet, allFrames, blockSize, sizingAllDeltas, originalAllDeltas,
		sizingProcessMotion, incrementCompletedCount); err != nil {
		return false, err
	}

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

	// あるボーンだけ上書きする
	for _, boneName := range []string{pmx.UPPER.String(), pmx.UPPER2.String(),
		pmx.ARM.Left(), pmx.ARM.Right(), pmx.NECK.String()} {
		if !sizingSet.SizingModel.Bones.ContainsByName(boneName) {
			continue
		}
		outputMotion.BoneFrames.Get(boneName).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
			processBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
			bf.Rotation = processBf.Rotation.Copy()
			outputMotion.BoneFrames.Get(boneName).Update(bf)

			return true
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
					outputMotion.InsertBoneFrame(upperBone.Name(), resultBf)
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

	if mlog.IsDebug() {
		outputVerboseMotion("上半身03", sizingSet.OutputMotionPath, outputMotion)
	}

	return nil
}

func updateUpper(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	upperRotations, upper2Rotations, upperCancelRotations, upper2CancelRotations []*mmath.MQuaternion,
) {
	for i, iFrame := range allFrames {
		frame := float32(iFrame)
		upperBf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingUpperBone().Name()).Get(frame)
		upperBf.Rotation = upperRotations[i]
		sizingProcessMotion.InsertBoneFrame(sizingSet.SizingUpperBone().Name(), upperBf)

		if sizingSet.SizingUpper2VanillaBone() != nil {
			upper2Bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingUpper2Bone().Name()).Get(frame)
			upper2Bf.Rotation = upper2Rotations[i]
			sizingProcessMotion.InsertBoneFrame(sizingSet.SizingUpper2Bone().Name(), upper2Bf)
		}

		{
			neckBf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingNeckRootBone().Name()).Get(frame)
			if upper2CancelRotations != nil && upper2CancelRotations[i] != nil {
				neckBf.Rotation = upper2CancelRotations[i].Muled(upperCancelRotations[i]).Muled(neckBf.FilledRotation())
			} else {
				neckBf.Rotation = upperCancelRotations[i].Muled(neckBf.FilledRotation())
			}
			sizingProcessMotion.InsertBoneFrame(sizingSet.SizingNeckRootBone().Name(), neckBf)
		}
		{
			for _, direction := range directions {
				armBf := sizingProcessMotion.BoneFrames.Get(pmx.ARM.StringFromDirection(direction)).Get(frame)
				if upper2CancelRotations != nil && upper2CancelRotations[i] != nil {
					armBf.Rotation = upper2CancelRotations[i].Muled(upperCancelRotations[i]).Muled(armBf.FilledRotation())
				} else {
					armBf.Rotation = upperCancelRotations[i].Muled(armBf.FilledRotation())
				}
				sizingProcessMotion.InsertBoneFrame(pmx.ARM.StringFromDirection(direction), armBf)
			}
		}

		if i > 0 && i%1000 == 0 {
			processLog("上半身補正03", sizingSet.Index, i, len(allFrames))
		}
	}

	if mlog.IsDebug() {
		outputVerboseMotion("上半身02", sizingSet.OutputMotionPath, sizingProcessMotion)
	}
}

func calculateUpperDistanceRange(bones *pmx.Bones, upperRootIndex, neckRootIndex int) float64 {
	distance := 0.0

	startIndex := neckRootIndex

	for range 100 {
		bone, err := bones.Get(startIndex)
		if err != nil || bone.ParentIndex == -1 {
			break
		}

		parentBone, err := bones.Get(bone.ParentIndex)
		if err != nil {
			break
		}

		distance += bone.Position.Distance(parentBone.Position)
		if parentBone.Index() == upperRootIndex {
			break
		}

		startIndex = parentBone.Index()
	}

	return distance

	// upperRootBone, err := bones.Get(upperRootIndex)
	// if err != nil {
	// 	return 0
	// }

	// neckRootBone, err := bones.Get(neckRootIndex)
	// if err != nil {
	// 	return 0
	// }

	// return upperRootBone.Position.Distance(neckRootBone.Position)
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
	if sizingSet.SizingUpper2Bone() != nil {
		upperBones = append(upperBones, sizingSet.SizingUpper2Bone())
	}
	if sizingSet.SizingUpperBone() != nil {
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
	sizingAllDeltas, originalAllDeltas []*delta.VmdDeltas, sizingProcessMotion *vmd.VmdMotion,
	incrementCompletedCount func(),
) error {
	if mlog.IsDebug() {
		outputVerboseMotion("上半身01", sizingSet.OutputMotionPath, sizingProcessMotion)
	}

	var upperDistance float64
	upperDistance, err := calculateUpperDistance(sizingSet)
	if err != nil {
		return err
	}

	upperRotations := make([]*mmath.MQuaternion, len(allFrames))
	upper2Rotations := make([]*mmath.MQuaternion, len(allFrames))
	upperCancelRotations := make([]*mmath.MQuaternion, len(allFrames))
	upper2CancelRotations := make([]*mmath.MQuaternion, len(allFrames))

	upperIkBone := createUpperIkBone(sizingSet)

	actualUpperRootPositions := make([]*mmath.MVec3, len(allFrames))
	actualNeckRootPositions := make([]*mmath.MVec3, len(allFrames))
	idealNeckRootPositions := make([]*mmath.MVec3, len(allFrames))

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

			if mlog.IsDebug() {
				idealNeckRootPositions[index] = sizingGlobalUpperPosition
				actualUpperRootPositions[index] = sizingAllDeltas[index].Bones.GetByName(pmx.UPPER_ROOT.String()).FilledGlobalPosition()
				actualNeckRootPositions[index] = sizingAllDeltas[index].Bones.GetByName(pmx.NECK_ROOT.String()).FilledGlobalPosition()
			}

			sizingUpperDeltas := deform.DeformIk(sizingSet.SizingConfigModel, sizingProcessMotion, sizingAllDeltas[index], float32(data), upperIkBone, sizingGlobalUpperPosition, trunk_upper_bone_names, false)

			upperRotations[index] = sizingUpperDeltas.Bones.GetByName(pmx.UPPER.String()).FilledFrameRotation()

			upperBf := sizingProcessMotion.BoneFrames.Get(pmx.UPPER.String()).Get(float32(data))
			upperCancelRotations[index] = upperRotations[index].Inverted().Muled(upperBf.FilledRotation())

			if sizingSet.SizingUpper2VanillaBone() != nil {
				upper2Rotations[index] = sizingUpperDeltas.Bones.GetByName(pmx.UPPER2.String()).FilledFrameRotation()

				upper2Bf := sizingProcessMotion.BoneFrames.Get(pmx.UPPER2.String()).Get(float32(data))
				upper2CancelRotations[index] = upper2Rotations[index].Inverted().Muled(upper2Bf.FilledRotation())
			}

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("上半身補正02", sizingSet.Index, iterIndex, allCount)
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
				bf.Position = idealNeckRootPositions[i]
				motion.InsertBoneFrame("上半身IK", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = actualUpperRootPositions[i]
				motion.InsertBoneFrame("上半身Root", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = actualNeckRootPositions[i]
				motion.InsertBoneFrame("上半身Tgt", bf)
			}
		}

		outputVerboseMotion("上半身02", sizingSet.OutputMotionPath, motion)
	}

	updateUpper(sizingSet, allFrames, sizingProcessMotion, upperRotations, upper2Rotations, upperCancelRotations, upper2CancelRotations)

	incrementCompletedCount()

	return nil
}

func checkBonesForSizingUpper(sizingSet *domain.SizingSet) (err error) {

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
