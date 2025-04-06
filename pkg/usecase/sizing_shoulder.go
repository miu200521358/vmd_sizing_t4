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

// SizingShoulder は肩補正処理を行います。
func SizingShoulder(sizingSet *domain.SizingSet, sizingSetCount int, incrementCompletedCount func()) (bool, error) {
	// 対象外の場合は何もせず終了
	if !sizingSet.IsSizingShoulder || sizingSet.CompletedSizingShoulder {
		return false, nil
	}

	originalMotion := sizingSet.OriginalMotion
	sizingProcessMotion, err := sizingSet.OutputMotion.Copy()
	if err != nil {
		return false, err
	}

	mlog.I(mi18n.T("肩補正開始", map[string]interface{}{"No": sizingSet.Index + 1}))

	// 処理対象ボーンチェック
	if err := checkBonesForSizingShoulder(sizingSet); err != nil {
		return false, err
	}

	allFrames := mmath.IntRanges(int(originalMotion.MaxFrame()) + 1)
	blockSize, _ := miter.GetBlockSize(len(allFrames) * sizingSetCount)

	// 元モデルのデフォーム結果を並列処理で取得
	originalAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.OriginalConfigModel, originalMotion, sizingSet, true, append(all_shoulder_bone_names[0], all_shoulder_bone_names[1]...), "肩補正01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	sizingAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel, sizingProcessMotion, sizingSet, true, append(all_shoulder_bone_names[0], all_shoulder_bone_names[1]...), "肩補正01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	if err := calculateAdjustedShoulder(sizingSet, allFrames, blockSize, sizingAllDeltas, originalAllDeltas, sizingProcessMotion, incrementCompletedCount); err != nil {
		return false, err
	}

	incrementCompletedCount()

	if err := updateShoulderResultMotion(sizingSet, allFrames, blockSize, sizingProcessMotion, incrementCompletedCount); err != nil {
		return false, err
	}

	incrementCompletedCount()

	sizingSet.CompletedSizingShoulder = true

	return true, nil
}

func createShoulderIkBone(sizingSet *domain.SizingSet, direction pmx.BoneDirection) *pmx.Bone {
	// 肩IK
	shoulderBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.SHOULDER.StringFromDirection(direction))
	armBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.ARM.StringFromDirection(direction))

	ikBone := pmx.NewBoneByName(fmt.Sprintf("%s%sIk", pmx.MLIB_PREFIX, pmx.SHOULDER.StringFromDirection(direction)))
	ikBone.Position = armBone.Position.Copy()
	ikBone.Ik = pmx.NewIk()
	ikBone.Ik.BoneIndex = armBone.Index()
	ikBone.Ik.LoopCount = 100
	ikBone.Ik.UnitRotation = &mmath.MVec3{X: 0.1, Y: 0.0, Z: 0.0}
	ikBone.Ik.Links = make([]*pmx.IkLink, 1)
	{
		ikBone.Ik.Links[0] = pmx.NewIkLink()
		ikBone.Ik.Links[0].BoneIndex = shoulderBone.Index()
	}

	return ikBone
}

func calculateAdjustedShoulder(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int,
	sizingAllDeltas, originalAllDeltas []*delta.VmdDeltas, sizingProcessMotion *vmd.VmdMotion,
	incrementCompletedCount func(),
) error {
	shoulderScales := calculateShoulderScale(sizingSet)

	shoulderIkBones := make([]*pmx.Bone, 2)
	for i, direction := range directions {
		shoulderIkBones[i] = createShoulderIkBone(sizingSet, direction)
	}

	armPositions := make([][]*mmath.MVec3, 2)
	armIdealPositions := make([][]*mmath.MVec3, 2)
	originalShoulderPositions := make([][]*mmath.MVec3, 2)
	shoulderPositions := make([][]*mmath.MVec3, 2)
	originalShoulderRotations := make([][]*mmath.MQuaternion, 2)
	shoulderRotations := make([][]*mmath.MQuaternion, 2)
	shoulderCancelRotations := make([][]*mmath.MQuaternion, 2)

	for i := range directions {
		armPositions[i] = make([]*mmath.MVec3, len(allFrames))
		armIdealPositions[i] = make([]*mmath.MVec3, len(allFrames))
		originalShoulderPositions[i] = make([]*mmath.MVec3, len(allFrames))
		shoulderPositions[i] = make([]*mmath.MVec3, len(allFrames))
		originalShoulderRotations[i] = make([]*mmath.MQuaternion, len(allFrames))
		shoulderRotations[i] = make([]*mmath.MQuaternion, len(allFrames))
		shoulderCancelRotations[i] = make([]*mmath.MQuaternion, len(allFrames))
	}

	err := miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			originalNeckRootDelta := originalAllDeltas[index].Bones.GetByName(pmx.NECK_ROOT.String())
			sizingNeckRootDelta := sizingAllDeltas[index].Bones.GetByName(pmx.NECK_ROOT.String())

			for i, direction := range directions {
				originalArmDelta := originalAllDeltas[index].Bones.GetByName(pmx.ARM.StringFromDirection(direction))
				sizingArmDelta := sizingAllDeltas[index].Bones.GetByName(pmx.ARM.StringFromDirection(direction))
				sizingShoulderDelta := sizingAllDeltas[index].Bones.GetByName(pmx.SHOULDER.StringFromDirection(direction))

				// 元の首根元から見た、腕のローカル位置
				originalArmLocalPosition := originalNeckRootDelta.FilledGlobalMatrix().Inverted().MulVec3(originalArmDelta.FilledGlobalPosition())
				// 腕のローカル位置を先モデルのスケールに合わせる
				sizingArmLocalPosition := originalArmLocalPosition.Muled(shoulderScales[i])
				// 元の首根元に先の腕のローカル位置を合わせたグローバル位置
				sizingArmIdealPosition := sizingNeckRootDelta.FilledGlobalMatrix().MulVec3(sizingArmLocalPosition)

				sizingArmDeltas := deform.DeformIk(sizingSet.SizingConfigModel, sizingProcessMotion, sizingAllDeltas[index], float32(data), shoulderIkBones[i], sizingArmIdealPosition, all_shoulder_bone_names[i], false)

				shoulderBf := sizingProcessMotion.BoneFrames.Get(pmx.SHOULDER.StringFromDirection(direction)).Get(float32(data))

				originalShoulderRotations[i][index] = shoulderBf.FilledRotation()
				shoulderRotations[i][index] = sizingArmDeltas.Bones.GetByName(pmx.SHOULDER.StringFromDirection(direction)).FilledFrameRotation()
				shoulderCancelRotations[i][index] = shoulderRotations[i][index].Inverted().Muled(originalShoulderRotations[i][index]).Normalized()

				if mlog.IsDebug() {
					armPositions[i][index] = sizingArmDelta.FilledGlobalPosition()
					armIdealPositions[i][index] = sizingArmIdealPosition
					originalShoulderPositions[i][index] = sizingShoulderDelta.FilledGlobalPosition()

					sizingDeformShoulderDelta := sizingArmDeltas.Bones.GetByName(pmx.SHOULDER.StringFromDirection(direction))
					shoulderPositions[i][index] = sizingDeformShoulderDelta.FilledGlobalPosition()
				}
			}

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("肩補正02", sizingSet.Index, iterIndex, allCount)
		})
	if err != nil {
		return err
	}

	if mlog.IsDebug() {
		motion := vmd.NewVmdMotion("")

		for i, iFrame := range allFrames {
			frame := float32(iFrame)
			for j, direction := range directions {
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = armPositions[j][i]
					motion.InsertRegisteredBoneFrame(fmt.Sprintf("先%s腕", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = armIdealPositions[j][i]
					motion.InsertRegisteredBoneFrame(fmt.Sprintf("先%s腕理想", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = originalShoulderPositions[j][i]
					bf.Rotation = originalShoulderRotations[j][i]
					motion.InsertRegisteredBoneFrame(fmt.Sprintf("先%s肩", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = shoulderPositions[j][i]
					bf.Rotation = shoulderRotations[j][i]
					motion.InsertRegisteredBoneFrame(fmt.Sprintf("先%s肩結果", direction.String()), bf)
				}
			}
		}

		outputVerboseMotion("肩02", sizingSet.OutputMotionPath, motion)
	}

	incrementCompletedCount()

	// 肩回転をサイジング先モーションに反映
	updateShoulder(sizingSet, allFrames, sizingProcessMotion, shoulderRotations, shoulderCancelRotations)

	return nil
}

// updateShoulder は、補正した下半身回転をサイジング先モーションに反映します。
func updateShoulder(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	shoulderRotations, shoulderCancelRotations [][]*mmath.MQuaternion,
) {
	for i, iFrame := range allFrames {
		frame := float32(iFrame)
		for j, direction := range directions {
			{
				bf := sizingProcessMotion.BoneFrames.Get(pmx.SHOULDER.StringFromDirection(direction)).Get(frame)
				bf.Rotation = shoulderRotations[j][i]
				sizingProcessMotion.InsertRegisteredBoneFrame(pmx.SHOULDER.StringFromDirection(direction), bf)
			}
			{
				bf := sizingProcessMotion.BoneFrames.Get(pmx.ARM.StringFromDirection(direction)).Get(frame)
				bf.Rotation = shoulderCancelRotations[j][i].Muled(bf.FilledRotation()).Normalized()
				sizingProcessMotion.InsertRegisteredBoneFrame(pmx.ARM.StringFromDirection(direction), bf)
			}
		}

		if i > 0 && i%1000 == 0 {
			processLog("肩補正03", sizingSet.Index, i, len(allFrames))
		}
	}

	if mlog.IsDebug() {
		outputVerboseMotion("肩03", sizingSet.OutputMotionPath, sizingProcessMotion)
	}
}

func calculateShoulderScale(sizingSet *domain.SizingSet) (shoulderScales []*mmath.MVec3) {
	shoulderScales = make([]*mmath.MVec3, 2)

	for i, direction := range directions {
		originalNeckRootBone, _ := sizingSet.OriginalConfigModel.Bones.GetByName(pmx.NECK_ROOT.String())
		originalArmBone, _ := sizingSet.OriginalConfigModel.Bones.GetByName(pmx.ARM.StringFromDirection(direction))

		sizingNeckRootBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.NECK_ROOT.String())
		sizingArmBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.ARM.StringFromDirection(direction))

		// 腕のX距離
		originalArmXLength := originalArmBone.Position.X - originalNeckRootBone.Position.X
		sizingArmXLength := sizingArmBone.Position.X - sizingNeckRootBone.Position.X

		shoulderScales[i] = &mmath.MVec3{
			X: sizingArmXLength / originalArmXLength,
			Y: 1.0,
			Z: 1.0,
		}
	}

	return shoulderScales
}

func updateShoulderResultMotion(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, sizingProcessMotion *vmd.VmdMotion,
	incrementCompletedCount func(),
) error {
	// 肩補正処理の結果をサイジング先モーションに反映
	sizingModel := sizingSet.SizingConfigModel
	outputMotion := sizingSet.OutputMotion

	activeFrames := getFrames(outputMotion, []string{pmx.SHOULDER.Left(), pmx.SHOULDER.Right()})

	// 肩はあるボーンだけ上書きする
	for _, boneName := range []string{
		pmx.SHOULDER.Left(), pmx.SHOULDER.Right(), pmx.ARM.Left(), pmx.ARM.Right(),
	} {
		if outputMotion.BoneFrames.Contains(boneName) {
			outputMotion.BoneFrames.Get(boneName).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
				processBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
				bf.Position = processBf.FilledPosition().Copy()
				bf.Rotation = processBf.FilledRotation().Copy()
				outputMotion.BoneFrames.Get(boneName).Update(bf)
				return true
			})
		} else {
			processBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(0)
			outputBf := outputMotion.BoneFrames.Get(boneName).Get(0)
			outputBf.Position = processBf.FilledPosition().Copy()
			outputBf.Rotation = processBf.FilledRotation().Copy()
			outputMotion.InsertRegisteredBoneFrame(boneName, outputBf)
		}
	}

	// 中間キーフレのズレをチェック
	armThreshold := 0.1

	err := miter.IterParallelByList(directions, 1, 1,
		func(dIndex int, direction pmx.BoneDirection) error {
			for tIndex, targetFrames := range [][]int{activeFrames, allFrames} {
				processAllDeltas, err := computeVmdDeltas(targetFrames, blockSize,
					sizingModel, sizingProcessMotion, sizingSet, true, all_shoulder_bone_names[dIndex], "肩補正01")
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
						sizingModel, outputMotion, sizingSet, true, all_shoulder_bone_names[dIndex], "")
					if err != nil {
						return err
					}

					// 腕の位置をチェック
					resultArmDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.ARM.StringFromDirection(direction))
					processArmDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.ARM.StringFromDirection(direction))

					// 各関節位置がズレている場合、元の回転を焼き込む
					if resultArmDelta.FilledGlobalPosition().Distance(processArmDelta.FilledGlobalPosition()) > armThreshold {
						boneName := pmx.SHOULDER.StringFromDirection(direction)
						processBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
						resultBf := outputMotion.BoneFrames.Get(boneName).Get(frame)
						resultBf.Rotation = processBf.FilledRotation().Copy()
						outputMotion.InsertRegisteredBoneFrame(boneName, resultBf)
					}

					if fIndex > 0 && fIndex%1000 == 0 {
						mlog.I(mi18n.T("肩補正04", map[string]interface{}{
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
		outputVerboseMotion("肩04", sizingSet.OutputMotionPath, outputMotion)
	}

	return err
}

func checkBonesForSizingShoulder(sizingSet *domain.SizingSet) (err error) {

	for _, v := range [][]interface{}{
		{sizingSet.OriginalNeckRootBone, pmx.NECK_ROOT.String(), false},
		{sizingSet.OriginalLeftShoulderBone, pmx.SHOULDER.Left(), true},
		{sizingSet.OriginalLeftArmBone, pmx.ARM.Left(), true},
		{sizingSet.OriginalLeftShoulderBone, pmx.SHOULDER.Right(), true},
		{sizingSet.OriginalLeftArmBone, pmx.ARM.Right(), true},
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

	for _, v := range [][]interface{}{
		{sizingSet.SizingNeckRootBone, pmx.NECK_ROOT.String(), false},
		{sizingSet.SizingLeftShoulderBone, pmx.SHOULDER.Left(), true},
		{sizingSet.SizingLeftArmBone, pmx.ARM.Left(), true},
		{sizingSet.SizingLeftShoulderBone, pmx.SHOULDER.Right(), true},
		{sizingSet.SizingLeftArmBone, pmx.ARM.Right(), true},
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
