package domain

import (
	"github.com/miu200521358/mlib_go/pkg/domain/pmx"
)

// GetOrFetchBone は指定されたモデルからボーンを取得するためのヘルパー関数です
// modelがnilの場合はnilを返します
// cachedBoneがnilの場合、fetchFuncを使用してボーンを取得しキャッシュします
func (ss *SizingSet) getOrFetchBone(model *pmx.PmxModel, boneCache map[string]*pmx.Bone, boneName string) *pmx.Bone {
	if model == nil {
		return nil
	}

	if cachedBone, ok := boneCache[boneName]; ok {
		return cachedBone
	}

	if cachedBone, err := model.Bones.GetByName(boneName); err == nil && cachedBone != nil {
		boneCache[boneName] = cachedBone
		return cachedBone
	}

	return nil
}

// --------------------------------------------------------------------
// OriginalConfigModel からボーンを取得するためのメソッド群

func (ss *SizingSet) OriginalCenterBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.CENTER.String())
}

func (ss *SizingSet) OriginalGrooveBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.GROOVE.String())
}

func (ss *SizingSet) OriginalBodyAxisBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.BODY_AXIS.String())
}

func (ss *SizingSet) OriginalTrunkRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.TRUNK_ROOT.String())
}

func (ss *SizingSet) OriginalUpperRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.UPPER_ROOT.String())
}

func (ss *SizingSet) OriginalUpperBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.UPPER.String())
}

func (ss *SizingSet) OriginalUpper2Bone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.UPPER2.String())
}

func (ss *SizingSet) OriginalNeckRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.NECK_ROOT.String())
}

func (ss *SizingSet) OriginalNeckBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.NECK.String())
}

func (ss *SizingSet) OriginalHeadBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.HEAD.String())
}

func (ss *SizingSet) OriginalLowerBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.LOWER.String())
}

func (ss *SizingSet) OriginalLowerRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.LOWER_ROOT.String())
}

func (ss *SizingSet) OriginalLegCenterBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.LEG_CENTER.String())
}

func (ss *SizingSet) OriginalLeftLegRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.LEG_ROOT.Left())
}

func (ss *SizingSet) OriginalRightLegRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.LEG_ROOT.Right())
}

func (ss *SizingSet) OriginalLeftLegIkParentBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.LEG_IK_PARENT.Left())
}

func (ss *SizingSet) OriginalLeftLegIkBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.LEG_IK.Left())
}

func (ss *SizingSet) OriginalLeftLegBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.LEG.Left())
}

func (ss *SizingSet) OriginalLeftKneeBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.KNEE.Left())
}

func (ss *SizingSet) OriginalLeftAnkleBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.ANKLE.Left())
}

func (ss *SizingSet) OriginalLeftAnkleDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.ANKLE_D.Left())
}

func (ss *SizingSet) OriginalLeftAnkleDGroundBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.ANKLE_D_GROUND.Left())
}

func (ss *SizingSet) OriginalLeftToeIkBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.TOE_IK.Left())
}

func (ss *SizingSet) OriginalLeftToeTailDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.TOE_T_D.Left())
}

func (ss *SizingSet) OriginalLeftHeelDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.HEEL_D.Left())
}

func (ss *SizingSet) OriginalLeftToePDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.TOE_P_D.Left())
}

func (ss *SizingSet) OriginalLeftToeCDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.TOE_C_D.Left())
}

func (ss *SizingSet) OriginalRightLegIkParentBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.LEG_IK_PARENT.Right())
}

func (ss *SizingSet) OriginalRightLegIkBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.LEG_IK.Right())
}

func (ss *SizingSet) OriginalRightLegBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.LEG.Right())
}

func (ss *SizingSet) OriginalRightKneeBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.KNEE.Right())
}

func (ss *SizingSet) OriginalRightAnkleBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.ANKLE.Right())
}

func (ss *SizingSet) OriginalRightAnkleDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.ANKLE_D.Right())
}

func (ss *SizingSet) OriginalRightAnkleDGroundBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.ANKLE_D_GROUND.Right())
}

func (ss *SizingSet) OriginalRightToeIkBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.TOE_IK.Right())
}

func (ss *SizingSet) OriginalRightToeTailDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.TOE_T_D.Right())
}

func (ss *SizingSet) OriginalRightHeelDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.HEEL_D.Right())
}

func (ss *SizingSet) OriginalRightToePDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.TOE_P_D.Right())
}

func (ss *SizingSet) OriginalRightToeCDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.TOE_C_D.Right())
}

func (ss *SizingSet) OriginalLeftShoulderBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.SHOULDER.Left())
}

func (ss *SizingSet) OriginalRightShoulderBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.SHOULDER.Right())
}

func (ss *SizingSet) OriginalLeftArmBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.ARM.Left())
}

func (ss *SizingSet) OriginalRightArmBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.ARM.Right())
}

func (ss *SizingSet) OriginalLeftElbowBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.ELBOW.Left())
}

func (ss *SizingSet) OriginalRightElbowBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.ELBOW.Right())
}

func (ss *SizingSet) OriginalLeftWristBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.WRIST.Left())
}

func (ss *SizingSet) OriginalRightWristBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.WRIST.Right())
}

func (ss *SizingSet) OriginalLeftWristTailBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.WRIST_TAIL.Left())
}

func (ss *SizingSet) OriginalRightWristTailBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.WRIST_TAIL.Right())
}

// --------------------------------------------------------------------
// SizingConfigModel からボーンを取得するためのメソッド群

func (ss *SizingSet) SizingCenterBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.CENTER.String())
}

func (ss *SizingSet) SizingGrooveBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.GROOVE.String())
}

func (ss *SizingSet) SizingBodyAxisBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.BODY_AXIS.String())
}

func (ss *SizingSet) SizingTrunkRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.TRUNK_ROOT.String())
}

func (ss *SizingSet) SizingUpperRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.UPPER_ROOT.String())
}

func (ss *SizingSet) SizingUpperBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.UPPER.String())
}

func (ss *SizingSet) SizingUpper2Bone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.UPPER2.String())
}

func (ss *SizingSet) SizingNeckRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.NECK_ROOT.String())
}

func (ss *SizingSet) SizingNeckBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.NECK.String())
}

func (ss *SizingSet) SizingHeadBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.HEAD.String())
}

func (ss *SizingSet) SizingLowerBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.LOWER.String())
}

func (ss *SizingSet) SizingLowerRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.LOWER_ROOT.String())
}

func (ss *SizingSet) SizingLegCenterBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.LEG_CENTER.String())
}

func (ss *SizingSet) SizingLeftLegRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.LEG_ROOT.Left())
}

func (ss *SizingSet) SizingRightLegRootBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.LEG_ROOT.Right())
}

func (ss *SizingSet) SizingLeftLegIkParentBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.LEG_IK_PARENT.Left())
}

func (ss *SizingSet) SizingLeftLegIkBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.LEG_IK.Left())
}

func (ss *SizingSet) SizingLeftLegBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.LEG.Left())
}

func (ss *SizingSet) SizingLeftKneeBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.KNEE.Left())
}

func (ss *SizingSet) SizingLeftAnkleBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.ANKLE.Left())
}

func (ss *SizingSet) SizingLeftAnkleDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.ANKLE_D.Left())
}

func (ss *SizingSet) SizingLeftAnkleDGroundBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.ANKLE_D_GROUND.Left())
}

func (ss *SizingSet) SizingLeftToeIkBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.TOE_IK.Left())
}

func (ss *SizingSet) SizingLeftToeTailDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.TOE_T_D.Left())
}

func (ss *SizingSet) SizingLeftHeelDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.HEEL_D.Left())
}

func (ss *SizingSet) SizingLeftToePDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.TOE_P_D.Left())
}

func (ss *SizingSet) SizingLeftToeCDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.TOE_C_D.Left())
}

func (ss *SizingSet) SizingRightLegIkParentBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.LEG_IK_PARENT.Right())
}

func (ss *SizingSet) SizingRightLegIkBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.LEG_IK.Right())
}

func (ss *SizingSet) SizingRightLegBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.LEG.Right())
}

func (ss *SizingSet) SizingRightKneeBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.KNEE.Right())
}

func (ss *SizingSet) SizingRightAnkleBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.ANKLE.Right())
}

func (ss *SizingSet) SizingRightAnkleDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.ANKLE_D.Right())
}

func (ss *SizingSet) SizingRightAnkleDGroundBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.ANKLE_D_GROUND.Right())
}

func (ss *SizingSet) SizingRightToeIkBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.TOE_IK.Right())
}

func (ss *SizingSet) SizingRightToeTailDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.TOE_T_D.Right())
}

func (ss *SizingSet) SizingRightHeelDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.HEEL_D.Right())
}

func (ss *SizingSet) SizingRightToePDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.TOE_P_D.Right())
}

func (ss *SizingSet) SizingRightToeCDBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.TOE_C_D.Right())
}

func (ss *SizingSet) SizingLeftShoulderBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.SHOULDER.Left())
}

func (ss *SizingSet) SizingRightShoulderBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.SHOULDER.Right())
}

func (ss *SizingSet) SizingLeftArmBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.ARM.Left())
}

func (ss *SizingSet) SizingRightArmBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.ARM.Right())
}

func (ss *SizingSet) SizingLeftElbowBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.ELBOW.Left())
}

func (ss *SizingSet) SizingRightElbowBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.ELBOW.Right())
}

func (ss *SizingSet) SizingLeftWristBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.WRIST.Left())
}

func (ss *SizingSet) SizingRightWristBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.WRIST.Right())
}

func (ss *SizingSet) SizingLeftWristTailBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.WRIST_TAIL.Left())
}

func (ss *SizingSet) SizingRightWristTailBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.WRIST_TAIL.Right())
}

// --------------------------------------------------------------------
// SizingModel からボーンを取得するためのメソッド群

func (ss *SizingSet) SizingGrooveVanillaBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingModel, ss.sizingVanillaBoneCache, pmx.GROOVE.String())
}

func (ss *SizingSet) SizingUpperVanillaBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingModel, ss.sizingVanillaBoneCache, pmx.UPPER.String())
}

func (ss *SizingSet) SizingUpper2VanillaBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingModel, ss.sizingVanillaBoneCache, pmx.UPPER2.String())
}

func (ss *SizingSet) SizingNeckRootVanillaBone() *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingModel, ss.sizingVanillaBoneCache, pmx.NECK_ROOT.String())
}
