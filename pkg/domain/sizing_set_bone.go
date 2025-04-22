package domain

import (
	"github.com/miu200521358/mlib_go/pkg/domain/pmx"
)

// GetOrFetchBone は指定されたモデルからボーンを取得するためのヘルパー関数です
// modelがnilの場合はnilを返します
// cachedBoneがnilの場合、fetchFuncを使用してボーンを取得しキャッシュします
func (ss *SizingSet) getOrFetchBone(model *pmx.PmxModel, cachedBone **pmx.Bone, boneName string) *pmx.Bone {
	if model == nil {
		return nil
	}

	if *cachedBone == nil {
		*cachedBone, _ = model.Bones.GetByName(boneName)
	}

	return *cachedBone
}

// --------------------------------------------------------------------
// OriginalConfigModel からボーンを取得するためのメソッド群

func (ss *SizingSet) OriginalCenterBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalCenterBone, pmx.CENTER.String())
}

func (ss *SizingSet) OriginalGrooveBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalGrooveBone, pmx.GROOVE.String())
}

func (ss *SizingSet) OriginalBodyAxisBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalBodyAxisBone, pmx.BODY_AXIS.String())
}

func (ss *SizingSet) OriginalTrunkRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalTrunkRootBone, pmx.TRUNK_ROOT.String())
}

func (ss *SizingSet) OriginalUpperRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalUpperRootBone, pmx.UPPER_ROOT.String())
}

func (ss *SizingSet) OriginalUpperBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalUpperBone, pmx.UPPER.String())
}

func (ss *SizingSet) OriginalUpper2Bone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalUpper2Bone, pmx.UPPER2.String())
}

func (ss *SizingSet) OriginalNeckRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalNeckRootBone, pmx.NECK_ROOT.String())
}

func (ss *SizingSet) OriginalNeckBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalNeckBone, pmx.NECK.String())
}

func (ss *SizingSet) OriginalHeadBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalHeadBone, pmx.HEAD.String())
}

func (ss *SizingSet) OriginalLowerBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLowerBone, pmx.LOWER.String())
}

func (ss *SizingSet) OriginalLowerRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLowerRootBone, pmx.LOWER_ROOT.String())
}

func (ss *SizingSet) OriginalLegCenterBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLegCenterBone, pmx.LEG_CENTER.String())
}

func (ss *SizingSet) OriginalLeftLegRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLeftLegRootBone, pmx.LEG_ROOT.Left())
}

func (ss *SizingSet) OriginalRightLegRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalRightLegRootBone, pmx.LEG_ROOT.Right())
}

func (ss *SizingSet) OriginalLeftLegIkParentBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLeftLegIkParentBone, pmx.LEG_IK_PARENT.Left())
}

func (ss *SizingSet) OriginalLeftLegIkBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLeftLegIkBone, pmx.LEG_IK.Left())
}

func (ss *SizingSet) OriginalLeftLegBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLeftLegBone, pmx.LEG.Left())
}

func (ss *SizingSet) OriginalLeftKneeBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLeftKneeBone, pmx.KNEE.Left())
}

func (ss *SizingSet) OriginalLeftAnkleBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLeftAnkleBone, pmx.ANKLE.Left())
}

func (ss *SizingSet) OriginalLeftAnkleDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLeftAnkleDBone, pmx.ANKLE_D.Left())
}

func (ss *SizingSet) OriginalLeftAnkleDGroundBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLeftAnkleDGroundBone, pmx.ANKLE_D_GROUND.Left())
}

func (ss *SizingSet) OriginalLeftToeIkBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLeftToeIkBone, pmx.TOE_IK.Left())
}

func (ss *SizingSet) OriginalLeftToeTailDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLeftToeTailDBone, pmx.TOE_T_D.Left())
}

func (ss *SizingSet) OriginalLeftHeelDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLeftHeelDBone, pmx.HEEL_D.Left())
}

func (ss *SizingSet) OriginalLeftToePDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLeftToePDBone, pmx.TOE_P_D.Left())
}

func (ss *SizingSet) OriginalLeftToeCDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLeftToeCDBone, pmx.TOE_C_D.Left())
}

func (ss *SizingSet) OriginalRightLegIkParentBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalRightLegIkParentBone, pmx.LEG_IK_PARENT.Right())
}

func (ss *SizingSet) OriginalRightLegIkBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalRightLegIkBone, pmx.LEG_IK.Right())
}

func (ss *SizingSet) OriginalRightLegBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalRightLegBone, pmx.LEG.Right())
}

func (ss *SizingSet) OriginalRightKneeBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalRightKneeBone, pmx.KNEE.Right())
}

func (ss *SizingSet) OriginalRightAnkleBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalRightAnkleBone, pmx.ANKLE.Right())
}

func (ss *SizingSet) OriginalRightAnkleDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalRightAnkleDBone, pmx.ANKLE_D.Right())
}

func (ss *SizingSet) OriginalRightAnkleDGroundBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalRightAnkleDGroundBone, pmx.ANKLE_D_GROUND.Right())
}

func (ss *SizingSet) OriginalRightToeIkBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalRightToeIkBone, pmx.TOE_IK.Right())
}

func (ss *SizingSet) OriginalRightToeTailDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalRightToeTailDBone, pmx.TOE_T_D.Right())
}

func (ss *SizingSet) OriginalRightHeelDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalRightHeelDBone, pmx.HEEL_D.Right())
}

func (ss *SizingSet) OriginalRightToePDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalRightToePDBone, pmx.TOE_P_D.Right())
}

func (ss *SizingSet) OriginalRightToeCDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalRightToeCDBone, pmx.TOE_C_D.Right())
}

func (ss *SizingSet) OriginalLeftShoulderBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLeftShoulderBone, pmx.SHOULDER.Left())
}

func (ss *SizingSet) OriginalRightShoulderBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalRightShoulderBone, pmx.SHOULDER.Right())
}

func (ss *SizingSet) OriginalLeftArmBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLeftArmBone, pmx.ARM.Left())
}

func (ss *SizingSet) OriginalRightArmBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalRightArmBone, pmx.ARM.Right())
}

func (ss *SizingSet) OriginalLeftElbowBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLeftElbowBone, pmx.ELBOW.Left())
}

func (ss *SizingSet) OriginalRightElbowBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalRightElbowBone, pmx.ELBOW.Right())
}

func (ss *SizingSet) OriginalLeftWristBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLeftWristBone, pmx.WRIST.Left())
}

func (ss *SizingSet) OriginalRightWristBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalRightWristBone, pmx.WRIST.Right())
}

func (ss *SizingSet) OriginalLeftWristTailBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalLeftWristTailBone, pmx.WRIST_TAIL.Left())
}

func (ss *SizingSet) OriginalRightWristTailBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, &ss.originalRightWristTailBone, pmx.WRIST_TAIL.Right())
}

// --------------------------------------------------------------------
// SizingConfigModel からボーンを取得するためのメソッド群

func (ss *SizingSet) SizingCenterBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingCenterBone, pmx.CENTER.String())
}

func (ss *SizingSet) SizingGrooveBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingGrooveBone, pmx.GROOVE.String())
}

func (ss *SizingSet) SizingBodyAxisBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingBodyAxisBone, pmx.BODY_AXIS.String())
}

func (ss *SizingSet) SizingTrunkRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingTrunkRootBone, pmx.TRUNK_ROOT.String())
}

func (ss *SizingSet) SizingUpperRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingUpperRootBone, pmx.UPPER_ROOT.String())
}

func (ss *SizingSet) SizingUpperBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingUpperBone, pmx.UPPER.String())
}

func (ss *SizingSet) SizingUpper2Bone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingUpper2Bone, pmx.UPPER2.String())
}

func (ss *SizingSet) SizingNeckRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingNeckRootBone, pmx.NECK_ROOT.String())
}

func (ss *SizingSet) SizingNeckBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingNeckBone, pmx.NECK.String())
}

func (ss *SizingSet) SizingHeadBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingHeadBone, pmx.HEAD.String())
}

func (ss *SizingSet) SizingLowerBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLowerBone, pmx.LOWER.String())
}

func (ss *SizingSet) SizingLowerRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLowerRootBone, pmx.LOWER_ROOT.String())
}

func (ss *SizingSet) SizingLegCenterBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLegCenterBone, pmx.LEG_CENTER.String())
}

func (ss *SizingSet) SizingLeftLegRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLeftLegRootBone, pmx.LEG_ROOT.Left())
}

func (ss *SizingSet) SizingRightLegRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingRightLegRootBone, pmx.LEG_ROOT.Right())
}

func (ss *SizingSet) SizingLeftLegIkParentBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLeftLegIkParentBone, pmx.LEG_IK_PARENT.Left())
}

func (ss *SizingSet) SizingLeftLegIkBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLeftLegIkBone, pmx.LEG_IK.Left())
}

func (ss *SizingSet) SizingLeftLegBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLeftLegBone, pmx.LEG.Left())
}

func (ss *SizingSet) SizingLeftKneeBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLeftKneeBone, pmx.KNEE.Left())
}

func (ss *SizingSet) SizingLeftAnkleBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLeftAnkleBone, pmx.ANKLE.Left())
}

func (ss *SizingSet) SizingLeftAnkleDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLeftAnkleDBone, pmx.ANKLE_D.Left())
}

func (ss *SizingSet) SizingLeftAnkleDGroundBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLeftAnkleDGroundBone, pmx.ANKLE_D_GROUND.Left())
}

func (ss *SizingSet) SizingLeftToeIkBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLeftToeIkBone, pmx.TOE_IK.Left())
}

func (ss *SizingSet) SizingLeftToeTailDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLeftToeTailDBone, pmx.TOE_T_D.Left())
}

func (ss *SizingSet) SizingLeftHeelDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLeftHeelDBone, pmx.HEEL_D.Left())
}

func (ss *SizingSet) SizingLeftToePDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLeftToePDBone, pmx.TOE_P_D.Left())
}

func (ss *SizingSet) SizingLeftToeCDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLeftToeCDBone, pmx.TOE_C_D.Left())
}

func (ss *SizingSet) SizingRightLegIkParentBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingRightLegIkParentBone, pmx.LEG_IK_PARENT.Right())
}

func (ss *SizingSet) SizingRightLegIkBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingRightLegIkBone, pmx.LEG_IK.Right())
}

func (ss *SizingSet) SizingRightLegBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingRightLegBone, pmx.LEG.Right())
}

func (ss *SizingSet) SizingRightKneeBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingRightKneeBone, pmx.KNEE.Right())
}

func (ss *SizingSet) SizingRightAnkleBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingRightAnkleBone, pmx.ANKLE.Right())
}

func (ss *SizingSet) SizingRightAnkleDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingRightAnkleDBone, pmx.ANKLE_D.Right())
}

func (ss *SizingSet) SizingRightAnkleDGroundBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingRightAnkleDGroundBone, pmx.ANKLE_D_GROUND.Right())
}

func (ss *SizingSet) SizingRightToeIkBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingRightToeIkBone, pmx.TOE_IK.Right())
}

func (ss *SizingSet) SizingRightToeTailDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingRightToeTailDBone, pmx.TOE_T_D.Right())
}

func (ss *SizingSet) SizingRightHeelDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingRightHeelDBone, pmx.HEEL_D.Right())
}

func (ss *SizingSet) SizingRightToePDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingRightToePDBone, pmx.TOE_P_D.Right())
}

func (ss *SizingSet) SizingRightToeCDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingRightToeCDBone, pmx.TOE_C_D.Right())
}

func (ss *SizingSet) SizingLeftShoulderBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLeftShoulderBone, pmx.SHOULDER.Left())
}

func (ss *SizingSet) SizingRightShoulderBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingRightShoulderBone, pmx.SHOULDER.Right())
}

func (ss *SizingSet) SizingLeftArmBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLeftArmBone, pmx.ARM.Left())
}

func (ss *SizingSet) SizingRightArmBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingRightArmBone, pmx.ARM.Right())
}

func (ss *SizingSet) SizingLeftElbowBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLeftElbowBone, pmx.ELBOW.Left())
}

func (ss *SizingSet) SizingRightElbowBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingRightElbowBone, pmx.ELBOW.Right())
}

func (ss *SizingSet) SizingLeftWristBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLeftWristBone, pmx.WRIST.Left())
}

func (ss *SizingSet) SizingRightWristBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingRightWristBone, pmx.WRIST.Right())
}

func (ss *SizingSet) SizingLeftWristTailBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingLeftWristTailBone, pmx.WRIST_TAIL.Left())
}

func (ss *SizingSet) SizingRightWristTailBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, &ss.sizingRightWristTailBone, pmx.WRIST_TAIL.Right())
}

// --------------------------------------------------------------------
// SizingModel からボーンを取得するためのメソッド群

func (ss *SizingSet) SizingGrooveVanillaBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingModel, &ss.sizingGrooveVanillaBone, pmx.GROOVE.String())
}

func (ss *SizingSet) SizingUpperVanillaBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingModel, &ss.sizingUpperVanillaBone, pmx.UPPER.String())
}

func (ss *SizingSet) SizingUpper2VanillaBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingModel, &ss.sizingUpper2VanillaBone, pmx.UPPER2.String())
}

func (ss *SizingSet) SizingNeckRootVanillaBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingModel, &ss.sizingNeckRootVanillaBone, pmx.NECK_ROOT.String())
}
