package usecase

import (
	"miu200521358/vmd_sizing_t4/pkg/domain"

	"github.com/miu200521358/mlib_go/pkg/config/merr"
	"github.com/miu200521358/mlib_go/pkg/config/mi18n"
	"github.com/miu200521358/mlib_go/pkg/config/mlog"
	"github.com/miu200521358/mlib_go/pkg/domain/delta"
	"github.com/miu200521358/mlib_go/pkg/domain/mmath"
	"github.com/miu200521358/mlib_go/pkg/domain/pmx"
	"github.com/miu200521358/mlib_go/pkg/domain/vmd"
	"github.com/miu200521358/mlib_go/pkg/infrastructure/miter"
)

// SizingShoulder は肩補正処理を行います。
func SizingShoulder(
	sizingSet *domain.SizingSet, moveScale *mmath.MVec3, sizingSetCount int, incrementCompletedCount func(),
) (bool, error) {
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
	originalAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.OriginalConfigModel, originalMotion, sizingSet, true, append(all_arm_bone_names[0], all_arm_bone_names[1]...), "肩補正01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	sizingAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel, sizingProcessMotion, sizingSet, true, append(all_arm_bone_names[0], all_arm_bone_names[1]...), "肩補正01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	if err := calculateAdjustedShoulder(sizingSet, allFrames, blockSize, sizingAllDeltas, originalAllDeltas, sizingProcessMotion, incrementCompletedCount); err != nil {
		return false, err
	}

	sizingSet.CompletedSizingShoulder = true

	return true, nil
}

func calculateAdjustedShoulder(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int,
	sizingAllDeltas, originalAllDeltas []*delta.VmdDeltas, sizingProcessMotion *vmd.VmdMotion,
	incrementCompletedCount func(),
) error {

	return nil
}

func calculateShoulderScale(sizingSet *domain.SizingSet) (*mmath.MVec3, error) {
	// 肩のスケールを取得
	originalArmDiff := sizingSet.OriginalLeftArmBone().Position.Subed(sizingSet.OriginalNeckRootBone().Position)

	sizingArmDiff := sizingSet.SizingLeftArmBone().Position.Subed(sizingSet.SizingNeckRootBone().Position)

	shoulderScale := sizingArmDiff.Dived(originalArmDiff)
	shoulderScale.Z = 1.0 // Z軸は1.0に固定

	return shoulderScale, nil
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
