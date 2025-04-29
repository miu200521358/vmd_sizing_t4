package domain

import "github.com/miu200521358/mlib_go/pkg/domain/pmx"

type CheckTrunkBoneType struct {
	CheckFunk func() *pmx.Bone
	BoneName  pmx.StandardBoneName
}

func (c *CheckTrunkBoneType) IsStandard() bool {
	config := pmx.BoneConfigFromName(c.BoneName.String())
	return config.IsStandard
}

type CheckDirectionBoneType struct {
	CheckFunk func(direction pmx.BoneDirection) *pmx.Bone
	BoneName  pmx.StandardBoneName
}

func (c *CheckDirectionBoneType) IsStandard(direction pmx.BoneDirection) bool {
	config := pmx.BoneConfigFromName(c.BoneName.StringFromDirection(direction))
	return config.IsStandard
}
