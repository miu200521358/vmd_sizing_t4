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

type SizingArmStanceUsecase struct {
}

func NewSizingArmStanceUsecase() *SizingArmStanceUsecase {
	return &SizingArmStanceUsecase{}
}

func (su *SizingArmStanceUsecase) Exec(
	sizingSet *domain.SizingSet, sizingSetCount int, incrementCompletedCount func(),
) (bool, error) {
	// 対象外の場合は何もせず終了
	if (!sizingSet.IsSizingArmStance || sizingSet.CompletedSizingArmStance) &&
		(!sizingSet.IsSizingFingerStance || sizingSet.CompletedSizingFingerStance) {
		return false, nil
	}

	// 処理対象ボーンチェック
	if err := su.checkBones(sizingSet); err != nil {
		return false, err
	}

	mlog.I(mi18n.T("腕指スタンス補正開始", map[string]interface{}{"No": sizingSet.Index + 1}))

	stanceRotations, err := su.createArmFingerStanceRotations(sizingSet)
	if err != nil {
		return false, err
	}

	if err := su.updateStanceRotations(sizingSet, stanceRotations, incrementCompletedCount); err != nil {
		return false, err
	}

	if sizingSet.IsSizingArmStance {
		sizingSet.CompletedSizingArmStance = true
	}

	if sizingSet.IsSizingFingerStance {
		sizingSet.CompletedSizingFingerStance = true
	}

	return true, nil
}

func (su *SizingArmStanceUsecase) updateStanceRotations(
	sizingSet *domain.SizingSet, stanceRotations map[int][]*mmath.MMat4, incrementCompletedCount func(),
) (err error) {
	count := int(sizingSet.OutputMotion.MaxFrame()) + 1

	boneRotations := make(map[string][]*mmath.MQuaternion)
	for dIndex := range directions {
		for _, boneName := range all_arm_stance_bone_names[dIndex] {
			boneRotations[boneName] = make([]*mmath.MQuaternion, count)
		}
	}

	err = miter.IterParallelByList(directions, 1, 1,
		func(dIndex int, direction pmx.BoneDirection) error {
			for _, boneName := range all_arm_stance_bone_names[dIndex] {
				bone, err := sizingSet.SizingConfigModel.Bones.GetByName(boneName)
				if err != nil {
					continue
				}

				if bone.Config().IsArm() && (!sizingSet.IsSizingArmStance || sizingSet.CompletedSizingArmStance) {
					// 既に腕が終わっていたらスルー
					continue
				}
				if bone.Config().IsFinger() && (!sizingSet.IsSizingFingerStance || sizingSet.CompletedSizingFingerStance) {
					// 既に指が終わっていたらスルー
					continue
				}

				sizingSet.OutputMotion.BoneFrames.Get(boneName).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
					if sizingSet.IsTerminate {
						return false
					}

					// 回転補正
					if _, ok := stanceRotations[bone.Index()]; ok {
						boneRotations[boneName][int(frame)] = stanceRotations[bone.Index()][0].Muled(
							bf.FilledRotation().ToMat4()).Muled(stanceRotations[bone.Index()][1]).Quaternion()
					}

					return true
				})

				if sizingSet.IsTerminate {
					return merr.TerminateError
				}
			}

			mlog.I(mi18n.T("腕指スタンス補正02", map[string]interface{}{
				"No":        sizingSet.Index + 1,
				"Direction": direction.String()}))

			incrementCompletedCount()

			return nil
		}, nil)

	if err != nil {
		return err
	}

	for boneName, rotations := range boneRotations {
		maxFrame := int(sizingSet.OutputMotion.BoneFrames.Get(boneName).MaxFrame())
		sizingSet.OutputMotion.BoneFrames.Get(boneName).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
			if sizingSet.IsTerminate {
				return false
			}

			iFrame := int(frame)

			if rotations[iFrame] == nil {
				return true
			}

			bf.Rotation = rotations[iFrame]
			sizingSet.OutputMotion.InsertBoneFrame(boneName, bf)

			if frame > 0 && iFrame%1000 == 0 {
				mlog.I(mi18n.T("腕指スタンス補正03", map[string]interface{}{
					"No":        sizingSet.Index + 1,
					"BoneName":  boneName,
					"IterIndex": fmt.Sprintf("%04d", iFrame),
					"AllCount":  fmt.Sprintf("%04d", maxFrame),
				}))
			}

			return true
		})
	}

	if mlog.IsDebug() {
		outputVerboseMotion("腕01", sizingSet.OutputMotionPath, sizingSet.OutputMotion)
	}

	incrementCompletedCount()

	return nil
}

func (su *SizingArmStanceUsecase) createArmFingerStanceRotations(sizingSet *domain.SizingSet) (stanceRotations map[int][]*mmath.MMat4, err error) {
	stanceRotations = make(map[int][]*mmath.MMat4)

	for i, direction := range directions {
		stanceBoneNames := make([][]string, 0)

		originalVmdDeltas, err := computeVmdDeltas([]int{0}, 1, sizingSet.OriginalConfigModel, vmd.InitialMotion, sizingSet, true, all_arm_stance_bone_names[i], "", nil)
		if err != nil {
			return nil, err
		}

		sizingVmdDeltas, err := computeVmdDeltas([]int{0}, 1, sizingSet.SizingConfigModel, vmd.InitialMotion, sizingSet, true, all_arm_stance_bone_names[i], "", nil)

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

			sizingFromBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(fromBoneName)

			originalTargetBone, _ := sizingSet.OriginalConfigModel.Bones.GetByName(targetBoneName)
			sizingTargetBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(targetBoneName)

			originalToBone, _ := sizingSet.OriginalConfigModel.Bones.GetByName(toBoneName)
			sizingToBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(toBoneName)

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
			} else if sizingTargetBone.Config().IsArm() {
				// 捩り成分を除去
				_, yzOffsetQuat := offsetQuat.SeparateTwistByAxis(sizingDirection)
				stanceRotations[sizingTargetBone.Index()][1] = yzOffsetQuat.ToMat4()
			} else if sizingTargetBone.Config().IsFinger() {
				// 指は開き具合だけを補正する
				_, yOffsetQuat, _ := offsetQuat.SeparateByAxis(sizingDirection)
				stanceRotations[sizingTargetBone.Index()][1] = yOffsetQuat.ToMat4()
			} else {
				stanceRotations[sizingTargetBone.Index()][1] = offsetQuat.ToMat4()
			}
		}
	}

	return stanceRotations, nil
}

func (su *SizingArmStanceUsecase) checkBones(sizingSet *domain.SizingSet) (err error) {

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
