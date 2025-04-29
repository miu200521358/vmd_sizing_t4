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

func (ss *SizingSet) OriginalLegRootBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.LEG_ROOT.StringFromDirection(direction))
}

func (ss *SizingSet) OriginalLegIkParentBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.LEG_IK_PARENT.StringFromDirection(direction))
}

func (ss *SizingSet) OriginalLegIkBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.LEG_IK.StringFromDirection(direction))
}

func (ss *SizingSet) OriginalLegBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.LEG.StringFromDirection(direction))
}

func (ss *SizingSet) OriginalKneeBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.KNEE.StringFromDirection(direction))
}

func (ss *SizingSet) OriginalKneeDBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.KNEE_D.StringFromDirection(direction))
}

func (ss *SizingSet) OriginalAnkleBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.ANKLE.StringFromDirection(direction))
}

func (ss *SizingSet) OriginalAnkleDBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.ANKLE_D.StringFromDirection(direction))
}

func (ss *SizingSet) OriginalAnkleDGroundBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.ANKLE_D_GROUND.StringFromDirection(direction))
}

func (ss *SizingSet) OriginalToeIkBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.TOE_IK.StringFromDirection(direction))
}

func (ss *SizingSet) OriginalToeTailDBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.TOE_T_D.StringFromDirection(direction))
}

func (ss *SizingSet) OriginalHeelDBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.HEEL_D.StringFromDirection(direction))
}

func (ss *SizingSet) OriginalToePDBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.TOE_P_D.StringFromDirection(direction))
}

func (ss *SizingSet) OriginalToeCDBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.TOE_C_D.StringFromDirection(direction))
}

func (ss *SizingSet) OriginalShoulderBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.SHOULDER.StringFromDirection(direction))
}

func (ss *SizingSet) OriginalArmBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.ARM.StringFromDirection(direction))
}

func (ss *SizingSet) OriginalElbowBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.ELBOW.StringFromDirection(direction))
}

func (ss *SizingSet) OriginalWristBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.WRIST.StringFromDirection(direction))
}

func (ss *SizingSet) OriginalWristTailBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.OriginalConfigModel, ss.originalBoneCache, pmx.WRIST_TAIL.StringFromDirection(direction))
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

func (ss *SizingSet) SizingLegRootBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.LEG_ROOT.StringFromDirection(direction))
}

func (ss *SizingSet) SizingLegIkParentBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.LEG_IK_PARENT.StringFromDirection(direction))
}

func (ss *SizingSet) SizingLegIkBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.LEG_IK.StringFromDirection(direction))
}

func (ss *SizingSet) SizingLegBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.LEG.StringFromDirection(direction))
}

func (ss *SizingSet) SizingKneeBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.KNEE.StringFromDirection(direction))
}

func (ss *SizingSet) SizingKneeDBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.KNEE_D.StringFromDirection(direction))
}

func (ss *SizingSet) SizingAnkleBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.ANKLE.StringFromDirection(direction))
}

func (ss *SizingSet) SizingAnkleDBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.ANKLE_D.StringFromDirection(direction))
}

func (ss *SizingSet) SizingAnkleDGroundBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.ANKLE_D_GROUND.StringFromDirection(direction))
}

func (ss *SizingSet) SizingToeIkBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.TOE_IK.StringFromDirection(direction))
}

func (ss *SizingSet) SizingToeTailDBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.TOE_T_D.StringFromDirection(direction))
}

func (ss *SizingSet) SizingHeelDBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.HEEL_D.StringFromDirection(direction))
}

func (ss *SizingSet) SizingToePDBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.TOE_P_D.StringFromDirection(direction))
}

func (ss *SizingSet) SizingToeCDBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.TOE_C_D.StringFromDirection(direction))
}

func (ss *SizingSet) SizingShoulderBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.SHOULDER.StringFromDirection(direction))
}

func (ss *SizingSet) SizingArmBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.ARM.StringFromDirection(direction))
}

func (ss *SizingSet) SizingElbowBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.ELBOW.StringFromDirection(direction))
}

func (ss *SizingSet) SizingWristBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.WRIST.StringFromDirection(direction))
}

func (ss *SizingSet) SizingWristTailBone(direction pmx.BoneDirection) *pmx.Bone {
	return ss.getOrFetchBone(ss.SizingConfigModel, ss.sizingBoneCache, pmx.WRIST_TAIL.StringFromDirection(direction))
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
