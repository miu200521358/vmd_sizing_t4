package domain

import "github.com/miu200521358/mlib_go/pkg/domain/pmx"

func (ss *SizingSet) OriginalCenterBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalCenterBone == nil {
		ss.originalCenterBone, _ = ss.OriginalConfigModel.Bones.GetCenter()
	}

	return ss.originalCenterBone
}

func (ss *SizingSet) OriginalGrooveBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalGrooveBone == nil {
		ss.originalGrooveBone, _ = ss.OriginalConfigModel.Bones.GetGroove()
	}

	return ss.originalGrooveBone
}

func (ss *SizingSet) OriginalLowerBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalLowerBone == nil {
		ss.originalLowerBone, _ = ss.OriginalConfigModel.Bones.GetLower()
	}

	return ss.originalLowerBone
}

func (ss *SizingSet) OriginalLeftLegIkParentBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalLeftLegIkParentBone == nil {
		ss.originalLeftLegIkParentBone, _ = ss.OriginalConfigModel.Bones.GetLegIkParent(pmx.BONE_DIRECTION_LEFT)
	}

	return ss.originalLeftLegIkParentBone
}

func (ss *SizingSet) OriginalLeftLegIkBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalLeftLegIkBone == nil {
		ss.originalLeftLegIkBone, _ = ss.OriginalConfigModel.Bones.GetLegIk(pmx.BONE_DIRECTION_LEFT)
	}

	return ss.originalLeftLegIkBone
}

func (ss *SizingSet) OriginalLeftLegBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalLeftLegBone == nil {
		ss.originalLeftLegBone, _ = ss.OriginalConfigModel.Bones.GetLeg(pmx.BONE_DIRECTION_LEFT)
	}

	return ss.originalLeftLegBone
}

func (ss *SizingSet) OriginalLeftKneeBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalLeftKneeBone == nil {
		ss.originalLeftKneeBone, _ = ss.OriginalConfigModel.Bones.GetKnee(pmx.BONE_DIRECTION_LEFT)
	}

	return ss.originalLeftKneeBone
}

func (ss *SizingSet) OriginalLeftAnkleBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalLeftAnkleBone == nil {
		ss.originalLeftAnkleBone, _ = ss.OriginalConfigModel.Bones.GetAnkle(pmx.BONE_DIRECTION_LEFT)
	}

	return ss.originalLeftAnkleBone
}

func (ss *SizingSet) OriginalLeftToeIkBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalLeftToeIkBone == nil {
		ss.originalLeftToeIkBone, _ = ss.OriginalConfigModel.Bones.GetToeIk(pmx.BONE_DIRECTION_LEFT)
	}

	return ss.originalLeftToeIkBone
}

func (ss *SizingSet) OriginalLeftToeTailBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalLeftToeTailBone == nil {
		ss.originalLeftToeTailBone, _ = ss.OriginalConfigModel.Bones.GetToeT(pmx.BONE_DIRECTION_LEFT)
	}

	return ss.originalLeftToeTailBone
}

func (ss *SizingSet) OriginalLeftHeelBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalLeftHeelBone == nil {
		ss.originalLeftHeelBone, _ = ss.OriginalConfigModel.Bones.GetHeel(pmx.BONE_DIRECTION_LEFT)
	}

	return ss.originalLeftHeelBone
}

func (ss *SizingSet) OriginalLeftToePBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalLeftToePBone == nil {
		ss.originalLeftToePBone, _ = ss.OriginalConfigModel.Bones.GetToeP(pmx.BONE_DIRECTION_LEFT)
	}

	return ss.originalLeftToePBone
}

func (ss *SizingSet) OriginalRightLegIkParentBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalRightLegIkParentBone == nil {
		ss.originalRightLegIkParentBone, _ = ss.OriginalConfigModel.Bones.GetLegIkParent(pmx.BONE_DIRECTION_RIGHT)
	}

	return ss.originalRightLegIkParentBone
}

func (ss *SizingSet) OriginalRightLegIkBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalRightLegIkBone == nil {
		ss.originalRightLegIkBone, _ = ss.OriginalConfigModel.Bones.GetLegIk(pmx.BONE_DIRECTION_RIGHT)
	}

	return ss.originalRightLegIkBone
}

func (ss *SizingSet) OriginalRightLegBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalRightLegBone == nil {
		ss.originalRightLegBone, _ = ss.OriginalConfigModel.Bones.GetLeg(pmx.BONE_DIRECTION_RIGHT)
	}

	return ss.originalRightLegBone
}

func (ss *SizingSet) OriginalRightKneeBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalRightKneeBone == nil {
		ss.originalRightKneeBone, _ = ss.OriginalConfigModel.Bones.GetKnee(pmx.BONE_DIRECTION_RIGHT)
	}

	return ss.originalRightKneeBone
}

func (ss *SizingSet) OriginalRightAnkleBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalRightAnkleBone == nil {
		ss.originalRightAnkleBone, _ = ss.OriginalConfigModel.Bones.GetAnkle(pmx.BONE_DIRECTION_RIGHT)
	}

	return ss.originalRightAnkleBone
}

func (ss *SizingSet) OriginalRightToeIkBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalRightToeIkBone == nil {
		ss.originalRightToeIkBone, _ = ss.OriginalConfigModel.Bones.GetToeIk(pmx.BONE_DIRECTION_RIGHT)
	}

	return ss.originalRightToeIkBone
}

func (ss *SizingSet) OriginalRightToeTailBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalRightToeTailBone == nil {
		ss.originalRightToeTailBone, _ = ss.OriginalConfigModel.Bones.GetToeT(pmx.BONE_DIRECTION_RIGHT)
	}

	return ss.originalRightToeTailBone
}

func (ss *SizingSet) OriginalRightHeelBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalRightHeelBone == nil {
		ss.originalRightHeelBone, _ = ss.OriginalConfigModel.Bones.GetHeel(pmx.BONE_DIRECTION_RIGHT)
	}

	return ss.originalRightHeelBone
}

func (ss *SizingSet) OriginalRightToePBone() *pmx.Bone {
	if ss.OriginalConfigModel == nil {
		return nil
	}

	if ss.originalRightToePBone == nil {
		ss.originalRightToePBone, _ = ss.OriginalConfigModel.Bones.GetToeP(pmx.BONE_DIRECTION_RIGHT)
	}

	return ss.originalRightToePBone
}

func (ss *SizingSet) SizingCenterBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingCenterBone == nil {
		ss.sizingCenterBone, _ = ss.SizingConfigModel.Bones.GetCenter()
	}

	return ss.sizingCenterBone
}

func (ss *SizingSet) SizingLowerBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingLowerBone == nil {
		ss.sizingLowerBone, _ = ss.SizingConfigModel.Bones.GetLower()
	}

	return ss.sizingLowerBone
}

func (ss *SizingSet) SizingGrooveBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingGrooveBone == nil {
		ss.sizingGrooveBone, _ = ss.SizingConfigModel.Bones.GetGroove()
	}

	return ss.sizingGrooveBone
}

func (ss *SizingSet) SizingLeftLegIkParentBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingLeftLegIkParentBone == nil {
		ss.sizingLeftLegIkParentBone, _ = ss.SizingConfigModel.Bones.GetLegIkParent(pmx.BONE_DIRECTION_LEFT)
	}

	return ss.sizingLeftLegIkParentBone
}

func (ss *SizingSet) SizingLeftLegIkBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingLeftLegIkBone == nil {
		ss.sizingLeftLegIkBone, _ = ss.SizingConfigModel.Bones.GetLegIk(pmx.BONE_DIRECTION_LEFT)
	}

	return ss.sizingLeftLegIkBone
}

func (ss *SizingSet) SizingLeftLegBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingLeftLegBone == nil {
		ss.sizingLeftLegBone, _ = ss.SizingConfigModel.Bones.GetLeg(pmx.BONE_DIRECTION_LEFT)
	}

	return ss.sizingLeftLegBone
}

func (ss *SizingSet) SizingLeftKneeBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingLeftKneeBone == nil {
		ss.sizingLeftKneeBone, _ = ss.SizingConfigModel.Bones.GetKnee(pmx.BONE_DIRECTION_LEFT)
	}

	return ss.sizingLeftKneeBone
}

func (ss *SizingSet) SizingLeftAnkleBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingLeftAnkleBone == nil {
		ss.sizingLeftAnkleBone, _ = ss.SizingConfigModel.Bones.GetAnkle(pmx.BONE_DIRECTION_LEFT)
	}

	return ss.sizingLeftAnkleBone
}

func (ss *SizingSet) SizingLeftToeIkBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingLeftToeIkBone == nil {
		ss.sizingLeftToeIkBone, _ = ss.SizingConfigModel.Bones.GetToeIk(pmx.BONE_DIRECTION_LEFT)
	}

	return ss.sizingLeftToeIkBone
}

func (ss *SizingSet) SizingLeftToeTailBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingLeftToeTailBone == nil {
		ss.sizingLeftToeTailBone, _ = ss.SizingConfigModel.Bones.GetToeT(pmx.BONE_DIRECTION_LEFT)
	}

	return ss.sizingLeftToeTailBone
}

func (ss *SizingSet) SizingLeftHeelBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingLeftHeelBone == nil {
		ss.sizingLeftHeelBone, _ = ss.SizingConfigModel.Bones.GetHeel(pmx.BONE_DIRECTION_LEFT)
	}

	return ss.sizingLeftHeelBone
}

func (ss *SizingSet) SizingLeftToePBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingLeftToePBone == nil {
		ss.sizingLeftToePBone, _ = ss.SizingConfigModel.Bones.GetToeP(pmx.BONE_DIRECTION_LEFT)
	}

	return ss.sizingLeftToePBone
}

func (ss *SizingSet) SizingRightLegIkParentBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingRightLegIkParentBone == nil {
		ss.sizingRightLegIkParentBone, _ = ss.SizingConfigModel.Bones.GetLegIkParent(pmx.BONE_DIRECTION_RIGHT)
	}

	return ss.sizingRightLegIkParentBone
}

func (ss *SizingSet) SizingRightLegIkBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingRightLegIkBone == nil {
		ss.sizingRightLegIkBone, _ = ss.SizingConfigModel.Bones.GetLegIk(pmx.BONE_DIRECTION_RIGHT)
	}

	return ss.sizingRightLegIkBone
}

func (ss *SizingSet) SizingRightLegBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingRightLegBone == nil {
		ss.sizingRightLegBone, _ = ss.SizingConfigModel.Bones.GetLeg(pmx.BONE_DIRECTION_RIGHT)
	}

	return ss.sizingRightLegBone
}

func (ss *SizingSet) SizingRightKneeBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingRightKneeBone == nil {
		ss.sizingRightKneeBone, _ = ss.SizingConfigModel.Bones.GetKnee(pmx.BONE_DIRECTION_RIGHT)
	}

	return ss.sizingRightKneeBone
}

func (ss *SizingSet) SizingRightAnkleBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingRightAnkleBone == nil {
		ss.sizingRightAnkleBone, _ = ss.SizingConfigModel.Bones.GetAnkle(pmx.BONE_DIRECTION_RIGHT)
	}

	return ss.sizingRightAnkleBone
}

func (ss *SizingSet) SizingRightToeIkBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingRightToeIkBone == nil {
		ss.sizingRightToeIkBone, _ = ss.SizingConfigModel.Bones.GetToeIk(pmx.BONE_DIRECTION_RIGHT)
	}

	return ss.sizingRightToeIkBone
}

func (ss *SizingSet) SizingRightToeTailBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingRightToeTailBone == nil {
		ss.sizingRightToeTailBone, _ = ss.SizingConfigModel.Bones.GetToeT(pmx.BONE_DIRECTION_RIGHT)
	}

	return ss.sizingRightToeTailBone
}

func (ss *SizingSet) SizingRightHeelBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingRightHeelBone == nil {
		ss.sizingRightHeelBone, _ = ss.SizingConfigModel.Bones.GetHeel(pmx.BONE_DIRECTION_RIGHT)
	}

	return ss.sizingRightHeelBone
}

func (ss *SizingSet) SizingRightToePBone() *pmx.Bone {
	if ss.SizingConfigModel == nil {
		return nil
	}

	if ss.sizingRightToePBone == nil {
		ss.sizingRightToePBone, _ = ss.SizingConfigModel.Bones.GetToeP(pmx.BONE_DIRECTION_RIGHT)
	}

	return ss.sizingRightToePBone
}

func (ss *SizingSet) SizingGrooveVanillaBone() *pmx.Bone {
	if ss.SizingModel == nil {
		return nil
	}

	if ss.sizingGrooveVanillaBone == nil {
		ss.sizingGrooveVanillaBone, _ = ss.SizingModel.Bones.GetGroove()
	}

	return ss.sizingGrooveVanillaBone
}
