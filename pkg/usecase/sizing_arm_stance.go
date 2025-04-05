package usecase

import (
	"fmt"
	"miu200521358/vmd_sizing_t4/pkg/domain"

	"github.com/miu200521358/mlib_go/pkg/config/merr"
	"github.com/miu200521358/mlib_go/pkg/config/mi18n"
	"github.com/miu200521358/mlib_go/pkg/config/mlog"
	"github.com/miu200521358/mlib_go/pkg/domain/mmath"
	"github.com/miu200521358/mlib_go/pkg/domain/pmx"
	"github.com/miu200521358/mlib_go/pkg/domain/vmd"
	"github.com/miu200521358/mlib_go/pkg/infrastructure/miter"
)

func SizingArmFingerStance(
	sizingSet *domain.SizingSet, sizingSetCount int, incrementCompletedCount func(),
) (bool, error) {
	// 対象外の場合は何もせず終了
	if (!sizingSet.IsSizingArmStance || sizingSet.CompletedSizingArmStance) &&
		(!sizingSet.IsSizingFingerStance || sizingSet.CompletedSizingFingerStance) {
		return false, nil
	}

	// 処理対象ボーンチェック
	if err := checkBonesForSizingArmFinger(sizingSet); err != nil {
		return false, err
	}

	mlog.I(mi18n.T("腕指スタンス補正開始", map[string]interface{}{"No": sizingSet.Index + 1}))

	stanceRotations, err := createArmFingerStanceRotations(sizingSet)
	if err != nil {
		return false, err
	}

	if err := updateStanceRotations(sizingSet, stanceRotations, incrementCompletedCount); err != nil {
		return false, err
	}

	return true, nil
}

func updateStanceRotations(
	sizingSet *domain.SizingSet, stanceRotations map[int][]*mmath.MMat4, incrementCompletedCount func(),
) (err error) {
	boneRotations := make(map[string][]*mmath.MQuaternion)
	for dIndex := range directions {
		for _, boneName := range all_arm_stance_bone_names[dIndex] {
			boneRotations[boneName] = make([]*mmath.MQuaternion, int(sizingSet.OutputMotion.MaxFrame()+1))
		}
	}

	err = miter.IterParallelByList(directions, 1, 1,
		func(dIndex int, direction pmx.BoneDirection) error {
			for _, boneName := range all_arm_stance_bone_names[dIndex] {
				sizingSet.OutputMotion.BoneFrames.Get(boneName).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
					if sizingSet.IsTerminate {
						return false
					}

					// 回転補正
					bone, err := sizingSet.SizingConfigModel.Bones.GetByName(boneName)
					if err != nil {
						return true
					}

					if _, ok := stanceRotations[bone.Index()]; ok {
						sizingRotation := bf.Rotation
						if sizingRotation == nil {
							sizingRotation = mmath.MQuaternionIdent
						}
						boneRotations[boneName][int(frame)] = stanceRotations[bone.Index()][0].Muled(sizingRotation.ToMat4()).Muled(stanceRotations[bone.Index()][1]).Quaternion()
					}

					return true
				})

				if sizingSet.IsTerminate {
					return merr.TerminateError
				}

				mlog.I(mi18n.T("腕指スタンス補正02", map[string]interface{}{
					"No":        sizingSet.Index + 1,
					"Direction": direction.String()}))

				incrementCompletedCount()
			}

			return nil
		}, nil)

	if err != nil {
		return err
	}

	for boneName, rotations := range boneRotations {
		sizingSet.OutputMotion.BoneFrames.Get(boneName).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
			if sizingSet.IsTerminate {
				return false
			}

			iFrame := int(frame)

			if rotations[iFrame] == nil {
				return true
			}

			bf.Rotation = rotations[iFrame]
			sizingSet.OutputMotion.InsertRegisteredBoneFrame(boneName, bf)

			if frame > 0 && iFrame%1000 == 0 {
				mlog.I(mi18n.T("腕指スタンス補正03", map[string]interface{}{
					"No":        sizingSet.Index + 1,
					"BoneName":  boneName,
					"IterIndex": fmt.Sprintf("%04d", iFrame),
					"AllCount":  fmt.Sprintf("%02d", len(rotations)),
				}))
			}

			return true
		})
	}

	if mlog.IsDebug() {
		outputVerboseMotion("足10", sizingSet.OutputMotionPath, sizingSet.OutputMotion)
	}

	return nil
}

func createArmFingerStanceRotations(sizingSet *domain.SizingSet) (stanceRotations map[int][]*mmath.MMat4, err error) {
	stanceRotations = make(map[int][]*mmath.MMat4)

	for i, direction := range directions {
		stanceBoneNames := make([][]string, 0)

		originalVmdDeltas, err := computeVmdDeltas([]int{0}, 1, sizingSet.OriginalConfigModel, vmd.InitialMotion, sizingSet, true, all_arm_stance_bone_names[i], "腕指スタンス補正01")
		if err != nil {
			return nil, err
		}

		sizingVmdDeltas, err := computeVmdDeltas([]int{0}, 1, sizingSet.SizingConfigModel, vmd.InitialMotion, sizingSet, true, all_arm_stance_bone_names[i], "腕指スタンス補正01")

		if sizingSet.IsSizingArmStance {
			// 腕スタンス補正対象
			stanceBoneNames = append(stanceBoneNames, []string{"", pmx.ARM.StringFromDirection(direction), pmx.ELBOW.StringFromDirection(direction)})
			stanceBoneNames = append(stanceBoneNames,
				[]string{pmx.ARM.StringFromDirection(direction), pmx.ELBOW.StringFromDirection(direction), pmx.WRIST.StringFromDirection(direction)})
			stanceBoneNames = append(stanceBoneNames,
				[]string{pmx.ELBOW.StringFromDirection(direction), pmx.WRIST.StringFromDirection(direction), pmx.WRIST_TAIL.StringFromDirection(direction)})
		}

		if sizingSet.IsSizingFingerStance {
			// 指スタンス補正対象
			stanceBoneNames = append(stanceBoneNames, []string{
				pmx.WRIST.StringFromDirection(direction), pmx.THUMB1.StringFromDirection(direction), pmx.THUMB2.StringFromDirection(direction)})
			stanceBoneNames = append(stanceBoneNames,
				[]string{pmx.THUMB1.StringFromDirection(direction), pmx.THUMB2.StringFromDirection(direction), pmx.THUMB_TAIL.StringFromDirection(direction)})
			stanceBoneNames = append(stanceBoneNames, []string{
				pmx.WRIST.StringFromDirection(direction), pmx.INDEX1.StringFromDirection(direction), pmx.INDEX2.StringFromDirection(direction)})
			stanceBoneNames = append(stanceBoneNames,
				[]string{pmx.INDEX1.StringFromDirection(direction), pmx.INDEX2.StringFromDirection(direction), pmx.INDEX3.StringFromDirection(direction)})
			stanceBoneNames = append(stanceBoneNames,
				[]string{pmx.INDEX2.StringFromDirection(direction), pmx.INDEX3.StringFromDirection(direction), pmx.INDEX_TAIL.StringFromDirection(direction)})
			stanceBoneNames = append(stanceBoneNames, []string{
				pmx.WRIST.StringFromDirection(direction), pmx.MIDDLE1.StringFromDirection(direction), pmx.MIDDLE2.StringFromDirection(direction)})
			stanceBoneNames = append(stanceBoneNames,
				[]string{pmx.MIDDLE1.StringFromDirection(direction), pmx.MIDDLE2.StringFromDirection(direction), pmx.MIDDLE3.StringFromDirection(direction)})
			stanceBoneNames = append(stanceBoneNames,
				[]string{pmx.MIDDLE2.StringFromDirection(direction), pmx.MIDDLE3.StringFromDirection(direction), pmx.MIDDLE_TAIL.StringFromDirection(direction)})
			stanceBoneNames = append(stanceBoneNames, []string{
				pmx.WRIST.StringFromDirection(direction), pmx.RING1.StringFromDirection(direction), pmx.RING2.StringFromDirection(direction)})
			stanceBoneNames = append(stanceBoneNames,
				[]string{pmx.RING1.StringFromDirection(direction), pmx.RING2.StringFromDirection(direction), pmx.RING3.StringFromDirection(direction)})
			stanceBoneNames = append(stanceBoneNames,
				[]string{pmx.RING2.StringFromDirection(direction), pmx.RING3.StringFromDirection(direction), pmx.RING_TAIL.StringFromDirection(direction)})
			stanceBoneNames = append(stanceBoneNames, []string{
				pmx.WRIST.StringFromDirection(direction), pmx.PINKY1.StringFromDirection(direction), pmx.PINKY2.StringFromDirection(direction)})
			stanceBoneNames = append(stanceBoneNames,
				[]string{pmx.PINKY1.StringFromDirection(direction), pmx.PINKY2.StringFromDirection(direction), pmx.PINKY_TAIL.StringFromDirection(direction)})
		}

		for _, boneNames := range stanceBoneNames {
			fromBoneName := boneNames[0]
			targetBoneName := boneNames[1]
			toBoneName := boneNames[2]

			var sizingFromBone *pmx.Bone
			if fromBoneName != "" && sizingSet.SizingConfigModel.Bones.ContainsByName(fromBoneName) {
				sizingFromBone, _ = sizingSet.SizingConfigModel.Bones.GetByName(fromBoneName)
			}

			var originalTargetBone, sizingTargetBone *pmx.Bone
			if targetBoneName != "" && sizingSet.OriginalConfigModel.Bones.ContainsByName(targetBoneName) &&
				sizingSet.SizingConfigModel.Bones.ContainsByName(targetBoneName) {
				originalTargetBone, _ = sizingSet.OriginalConfigModel.Bones.GetByName(targetBoneName)
				sizingTargetBone, _ = sizingSet.SizingConfigModel.Bones.GetByName(targetBoneName)
			}

			var originalToBone, sizingToBone *pmx.Bone
			if toBoneName != "" && sizingSet.OriginalConfigModel.Bones.ContainsByName(toBoneName) &&
				sizingSet.SizingConfigModel.Bones.ContainsByName(toBoneName) {
				originalToBone, _ = sizingSet.OriginalConfigModel.Bones.GetByName(toBoneName)
				sizingToBone, _ = sizingSet.SizingConfigModel.Bones.GetByName(toBoneName)
			}

			if originalTargetBone == nil || sizingTargetBone == nil ||
				originalToBone == nil || sizingToBone == nil {
				continue
			}

			if _, ok := stanceRotations[sizingTargetBone.Index()]; !ok {
				stanceRotations[sizingTargetBone.Index()] = make([]*mmath.MMat4, 2)
			}

			if sizingFromBone != nil {
				if _, ok := stanceRotations[sizingFromBone.Index()]; ok {
					stanceRotations[sizingTargetBone.Index()][0] = stanceRotations[sizingFromBone.Index()][1].Inverted()
				} else {
					stanceRotations[sizingTargetBone.Index()][0] = mmath.NewMMat4()
				}
			} else {
				stanceRotations[sizingTargetBone.Index()][0] = mmath.NewMMat4()
			}

			// 元モデルのボーン傾き(デフォーム後)
			originalDirection := originalVmdDeltas[0].Bones.Get(originalTargetBone.Index()).FilledGlobalPosition().
				Subed(originalVmdDeltas[0].Bones.Get(originalToBone.Index()).FilledGlobalPosition()).Normalized()
			originalSlopeMat := originalDirection.ToLocalMat()

			// サイジング先モデルのボーン傾き(デフォーム後)
			sizingDirection := sizingVmdDeltas[0].Bones.Get(sizingTargetBone.Index()).FilledGlobalPosition().
				Subed(sizingVmdDeltas[0].Bones.Get(sizingToBone.Index()).FilledGlobalPosition()).Normalized()
			sizingSlopeMat := sizingDirection.ToLocalMat()
			// 傾き補正
			offsetQuat := sizingSlopeMat.Muled(originalSlopeMat.Inverted()).Inverted().Quaternion()

			if offsetQuat.IsIdent() {
				stanceRotations[sizingTargetBone.Index()][1] = mmath.NewMMat4()
			} else {
				_, yzOffsetQuat := offsetQuat.SeparateTwistByAxis(sizingDirection)
				stanceRotations[sizingTargetBone.Index()][1] = yzOffsetQuat.ToMat4()
			}
		}
	}

	return stanceRotations, nil
}

func checkBonesForSizingArmFinger(sizingSet *domain.SizingSet) (err error) {

	for _, v := range [][]interface{}{
		{sizingSet.OriginalLeftShoulderBone, pmx.SHOULDER.Left(), true},
		{sizingSet.OriginalLeftArmBone, pmx.ARM.Left(), true},
		{sizingSet.OriginalLeftElbowBone, pmx.ELBOW.Left(), true},
		{sizingSet.OriginalLeftWristBone, pmx.WRIST.Left(), true},
		{sizingSet.OriginalLeftWristTailBone, pmx.WRIST_TAIL.Left(), false},
		{sizingSet.OriginalRightShoulderBone, pmx.SHOULDER.Right(), true},
		{sizingSet.OriginalRightArmBone, pmx.ARM.Right(), true},
		{sizingSet.OriginalRightElbowBone, pmx.ELBOW.Right(), true},
		{sizingSet.OriginalRightWristBone, pmx.WRIST.Right(), true},
		{sizingSet.OriginalRightWristTailBone, pmx.WRIST_TAIL.Right(), false},
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
		{sizingSet.SizingLeftShoulderBone, pmx.SHOULDER.Left(), true},
		{sizingSet.SizingLeftArmBone, pmx.ARM.Left(), true},
		{sizingSet.SizingLeftElbowBone, pmx.ELBOW.Left(), true},
		{sizingSet.SizingLeftWristBone, pmx.WRIST.Left(), true},
		{sizingSet.SizingLeftWristTailBone, pmx.WRIST_TAIL.Left(), false},
		{sizingSet.SizingRightShoulderBone, pmx.SHOULDER.Right(), true},
		{sizingSet.SizingRightArmBone, pmx.ARM.Right(), true},
		{sizingSet.SizingRightElbowBone, pmx.ELBOW.Right(), true},
		{sizingSet.SizingRightWristBone, pmx.WRIST.Right(), true},
		{sizingSet.SizingRightWristTailBone, pmx.WRIST_TAIL.Right(), false},
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
