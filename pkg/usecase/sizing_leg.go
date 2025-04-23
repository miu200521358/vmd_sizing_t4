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

type SizingLegUsecase struct {
}

func NewSizingLegUsecase() *SizingLegUsecase {
	return &SizingLegUsecase{}
}

// SizingLeg は、足補正処理を行います。
func (su *SizingLegUsecase) Exec(
	sizingSet *domain.SizingSet, moveScale *mmath.MVec3, sizingSetCount int, incrementCompletedCount func(),
) (bool, error) {
	// 対象外の場合は何もせず終了
	if !sizingSet.IsSizingLeg || sizingSet.CompletedSizingLeg {
		return false, nil
	}

	originalMotion := sizingSet.OriginalMotion
	sizingProcessMotion, err := sizingSet.OutputMotion.Copy()
	if err != nil {
		return false, err
	}

	mlog.I(mi18n.T("足補正開始", map[string]interface{}{"No": sizingSet.Index + 1}))

	// 処理対象ボーンチェック
	if err := su.checkBones(sizingSet); err != nil {
		return false, err
	}

	allFrames := mmath.IntRanges(int(originalMotion.MaxFrame()) + 1)
	blockSize, _ := miter.GetBlockSize(len(allFrames) * sizingSetCount)

	// [焼き込み] -----------------------

	// 元モデルのデフォーム結果を並列処理で取得
	originalAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.OriginalConfigModel,
		originalMotion, sizingSet, true, all_lower_leg_bone_names, "足補正01", incrementCompletedCount)
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	// 元モデルのモーフデフォーム結果を並列処理で取得
	originalMorphAllDeltas, err := computeMorphVmdDeltas(allFrames, blockSize, sizingSet.OriginalConfigModel,
		originalMotion, sizingSet, all_lower_leg_bone_names, "足補正01", incrementCompletedCount)
	if err != nil {
		return false, err
	}

	// 先モデルのモーフデフォーム結果を並列処理で取得
	sizingMorphAllDeltas, err := computeMorphVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel,
		originalMotion, sizingSet, all_lower_leg_bone_names, "足補正01", incrementCompletedCount)
	if err != nil {
		return false, err
	}

	// TOE_IK キーフレームのリセット
	sizingProcessMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.TOE_IK.Left()))
	sizingProcessMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.TOE_IK.Right()))

	// サイジング先モデルに対して FK 焼き込み処理
	if err := su.updateLegFK(sizingSet, sizingProcessMotion, originalAllDeltas, incrementCompletedCount); err != nil {
		return false, err
	}

	if mlog.IsDebug() {
		su.insertIKFrames(sizingSet, sizingProcessMotion, false)
		outputVerboseMotion("足01", sizingSet.OutputMotionPath, sizingProcessMotion)
	}

	// [下半身] -----------------------

	{
		// 先モデルの足中心までのデフォーム結果を並列処理で取得
		sizingLowerAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel,
			sizingProcessMotion, sizingSet, true, trunk_lower_bone_names, "足補正01", incrementCompletedCount)
		if err != nil {
			return false, err
		}

		// 下半身補正を実施
		lowerRotations, err := su.calculateAdjustedLower(sizingSet, allFrames, blockSize,
			originalAllDeltas, sizingLowerAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas,
			sizingProcessMotion, incrementCompletedCount, "足02")
		if err != nil {
			return false, err
		}

		// 下半身回転をサイジング先モーションに反映
		su.updateLower(sizingSet, allFrames, sizingProcessMotion, lowerRotations, incrementCompletedCount)

		if mlog.IsDebug() {
			outputVerboseMotion("足03", sizingSet.OutputMotionPath, sizingProcessMotion)
		}
	}

	// [足IK] -----------------------

	lowerBoneNames := su.createLowerBoneNames(sizingSet)

	{
		// 先モデルの足のIK OFF状態でのデフォーム結果を並列処理で取得
		sizingLegIkAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel,
			sizingProcessMotion, sizingSet, false, lowerBoneNames, "足補正01", incrementCompletedCount)
		if err != nil {
			return false, err
		}

		// 足IK 補正処理
		legIkPositions, legRotations, err := su.calculateAdjustedLegIK(sizingSet, allFrames, blockSize,
			originalAllDeltas, sizingLegIkAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas,
			sizingProcessMotion, incrementCompletedCount, "足04")
		if err != nil {
			return false, err
		}

		// 足IKの位置と足の角度 をサイジング先モーションに反映
		su.updateLegIkAndFk(sizingSet, allFrames, sizingProcessMotion,
			legIkPositions, legRotations, incrementCompletedCount)

		if mlog.IsDebug() {
			outputVerboseMotion("足05", sizingSet.OutputMotionPath, sizingProcessMotion)
		}
	}

	// [センター] -----------------------

	{
		// 先モデルの足FK補正デフォーム結果を並列処理で取得
		sizingCenterAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel,
			sizingProcessMotion, sizingSet, false, lowerBoneNames, "足補正01", incrementCompletedCount)
		if err != nil {
			return false, err
		}

		// センター・グルーブ補正を実施
		rootPositions, centerPositions, groovePositions, leftLegParentPositions, rightLegParentPositions, err :=
			su.calculateAdjustedCenter(sizingSet, allFrames, blockSize, moveScale,
				originalAllDeltas, sizingCenterAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas,
				sizingProcessMotion, incrementCompletedCount, "足06")
		if err != nil {
			return false, err
		}

		// センター・グルーブ位置をサイジング先モーションに反映
		su.updateCenter(sizingSet, allFrames, sizingProcessMotion,
			rootPositions, centerPositions, groovePositions, leftLegParentPositions, rightLegParentPositions,
			incrementCompletedCount)

		if mlog.IsDebug() {
			outputVerboseMotion("足07", sizingSet.OutputMotionPath, sizingProcessMotion)
		}
	}

	// [足IK2回目] -----------------------

	{
		// 先モデルの足FK補正デフォーム結果を並列処理で取得
		sizingLegIk2AllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel,
			sizingProcessMotion, sizingSet, false, lowerBoneNames, "足補正01", incrementCompletedCount)
		if err != nil {
			return false, err
		}

		// 足IK 補正処理
		legIkPositions, legIkRotations, legRotations, kneeRotations, ankleRotations, err :=
			su.calculateAdjustedLegIK2(sizingSet, allFrames, blockSize,
				originalAllDeltas, sizingLegIk2AllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas,
				sizingProcessMotion, incrementCompletedCount, "足08")
		if err != nil {
			return false, err
		}

		// 足IKの位置と足の角度 をサイジング先モーションに反映
		su.updateLegIkAndFk2(sizingSet, allFrames, sizingProcessMotion, legIkPositions,
			legIkRotations, legRotations, kneeRotations, ankleRotations, incrementCompletedCount)

		if mlog.IsDebug() {
			outputVerboseMotion("足09", sizingSet.OutputMotionPath, sizingProcessMotion)
		}
	}

	// [結果出力] -----------------------

	{
		// 足補正処理の結果をサイジング先モーションに反映
		if err = su.updateOutputMotion(
			sizingSet, allFrames, blockSize, sizingProcessMotion, "足10",
			incrementCompletedCount,
		); err != nil {
			return false, err
		}

		if mlog.IsDebug() {
			outputVerboseMotion("足11", sizingSet.OutputMotionPath, sizingSet.OutputMotion)
		}

		// 足IKのオフセットを元に戻す
		su.updateLegIkOffset(sizingSet, allFrames)

		if mlog.IsDebug() {
			outputVerboseMotion("足12", sizingSet.OutputMotionPath, sizingSet.OutputMotion)
		}
	}

	sizingSet.CompletedSizingLeg = true

	return true, nil
}

// updateLegFK は、デフォーム結果から FK 回転をサイジング先モーションに焼き込みます。
func (su *SizingLegUsecase) updateLegFK(
	sizingSet *domain.SizingSet, sizingProcessMotion *vmd.VmdMotion, allDeltas []*delta.VmdDeltas,
	incrementCompletedCount func(),
) error {
	for i, vmdDeltas := range allDeltas {
		if sizingSet.IsTerminate {
			return merr.TerminateError
		}
		// 足・ひざ・足首の回転補正
		for _, boneName := range []string{
			pmx.LEG.Left(), pmx.LEG.Right(), pmx.KNEE.Left(), pmx.KNEE.Right(), pmx.ANKLE.Left(), pmx.ANKLE.Right(),
		} {
			boneDelta := vmdDeltas.Bones.GetByName(boneName)
			if boneDelta == nil {
				continue
			}
			bf := sizingProcessMotion.BoneFrames.Get(boneName).Get(boneDelta.Frame)
			bf.Rotation = boneDelta.FilledFrameRotation()
			sizingProcessMotion.InsertBoneFrame(boneName, bf)
		}

		if i > 0 && i%1000 == 0 {
			processLog("足補正02", sizingSet.Index, i, len(allDeltas))
		}

		incrementCompletedCount()
	}

	return nil
}

// insertIKFrames は、verbose モード時に IK フレームを挿入して中間結果を出力します。
func (su *SizingLegUsecase) insertIKFrames(sizingSet *domain.SizingSet, sizingProcessMotion *vmd.VmdMotion, enabled bool) {
	sizingProcessMotion.IkFrames = vmd.NewIkFrames()

	kf := vmd.NewIkFrame(0)
	// 左足
	kf.IkList = append(kf.IkList, su.newIkEnableFrameWithBone(sizingSet.SizingLeftLegIkBone().Name(), enabled))
	kf.IkList = append(kf.IkList, su.newIkEnableFrameWithBone(sizingSet.SizingLeftToeIkBone().Name(), enabled))
	// 右足
	kf.IkList = append(kf.IkList, su.newIkEnableFrameWithBone(sizingSet.SizingRightLegIkBone().Name(), enabled))
	kf.IkList = append(kf.IkList, su.newIkEnableFrameWithBone(sizingSet.SizingRightToeIkBone().Name(), enabled))
	sizingProcessMotion.AppendIkFrame(kf)
}

// newIkEnableFrameWithBone は、与えられたボーン名と有効フラグから新しい IK 有効フレームを生成するヘルパー関数です。
func (su *SizingLegUsecase) newIkEnableFrameWithBone(boneName string, enabled bool) *vmd.IkEnabledFrame {
	// 0 はフレーム番号の初期値として設定（必要に応じて変更してください）
	frame := vmd.NewIkEnableFrame(0)
	frame.BoneName = boneName
	frame.Enabled = enabled
	return frame
}

func (su *SizingLegUsecase) createLowerIkBone(sizingSet *domain.SizingSet, direction pmx.BoneDirection) *pmx.Bone {
	lowerBone := sizingSet.SizingLowerBone()
	var ikTargetBone *pmx.Bone
	switch direction {
	case pmx.BONE_DIRECTION_TRUNK:
		ikTargetBone = sizingSet.SizingLegCenterBone()
	default:
		ikTargetBone, _ = sizingSet.SizingConfigModel.Bones.GetLeg(direction)
	}

	// 下半身IK
	ikBone := pmx.NewBoneByName(fmt.Sprintf("%s%sIk", pmx.MLIB_PREFIX, lowerBone.Name()))
	ikBone.Position = ikTargetBone.Position.Copy()
	ikBone.Ik = pmx.NewIk()
	ikBone.Ik.BoneIndex = ikTargetBone.Index()
	ikBone.Ik.LoopCount = 100
	ikBone.Ik.UnitRotation = &mmath.MVec3{X: 0.1, Y: 0.0, Z: 0.0}
	ikBone.Ik.Links = make([]*pmx.IkLink, 0)
	for _, parentBoneIndex := range ikTargetBone.ParentBoneIndexes {
		link := pmx.NewIkLink()
		link.BoneIndex = parentBoneIndex
		if parentBoneIndex != lowerBone.Index() {
			// 下半身以外は動かさない
			link.AngleLimit = true
		}
		ikBone.Ik.Links = append(ikBone.Ik.Links, link)

		if parentBoneIndex == lowerBone.Index() {
			// 下半身までいったら終了
			break
		}
	}

	return ikBone
}

// calculateAdjustedCenter は、センターおよびグルーブの位置補正を並列処理で計算します。
func (su *SizingLegUsecase) calculateAdjustedLower(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int,
	originalAllDeltas, sizingAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas []*delta.VmdDeltas,
	sizingProcessMotion *vmd.VmdMotion, incrementCompletedCount func(), verboseMotionName string,
) (sizingLowerResultRotations []*mmath.MQuaternion, err error) {
	originalLowerInitialPositions := make([]*mmath.MVec3, len(allFrames))
	originalLowerInitialRotations := make([]*mmath.MQuaternion, len(allFrames))

	originalLegCenterInitialPositions := make([]*mmath.MVec3, len(allFrames))
	originalLeftLegInitialPositions := make([]*mmath.MVec3, len(allFrames))
	originalRightLegInitialPositions := make([]*mmath.MVec3, len(allFrames))

	sizingLegCenterIdealPositions := make([]*mmath.MVec3, len(allFrames))

	sizingLowerInitialPositions := make([]*mmath.MVec3, len(allFrames))
	sizingLowerInitialRotations := make([]*mmath.MQuaternion, len(allFrames))
	sizingLegCenterInitialPositions := make([]*mmath.MVec3, len(allFrames))
	sizingLeftLegInitialPositions := make([]*mmath.MVec3, len(allFrames))
	sizingRightLegInitialPositions := make([]*mmath.MVec3, len(allFrames))

	sizingLegCenterResultPositions := make([]*mmath.MVec3, len(allFrames))
	sizingLeftLegResultPositions := make([]*mmath.MVec3, len(allFrames))
	sizingRightLegResultPositions := make([]*mmath.MVec3, len(allFrames))
	sizingLowerResultPositions := make([]*mmath.MVec3, len(allFrames))
	sizingLowerResultRotations = make([]*mmath.MQuaternion, len(allFrames))

	lowerIkBone := su.createLowerIkBone(sizingSet, pmx.BONE_DIRECTION_TRUNK)

	err = miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			// 下半身から足中心の傾き
			originalMorphLowerDelta := originalMorphAllDeltas[index].Bones.GetByName(pmx.LOWER.String())
			originalMorphLegCenterDelta := originalMorphAllDeltas[index].Bones.GetByName(pmx.LEG_CENTER.String())
			sizingMorphLowerDelta := sizingMorphAllDeltas[index].Bones.GetByName(pmx.LOWER.String())
			sizingMorphLegCenterDelta := sizingMorphAllDeltas[index].Bones.GetByName(pmx.LEG_CENTER.String())

			// 下半身と足中心の相対位置
			originalMorphLegCenterLocalPosition := originalMorphLowerDelta.FilledGlobalMatrix().Inverted().MulVec3(
				originalMorphLegCenterDelta.FilledGlobalPosition())
			sizingMorphLegCenterLocalPosition := sizingMorphLowerDelta.FilledGlobalMatrix().Inverted().MulVec3(
				sizingMorphLegCenterDelta.FilledGlobalPosition())

			// 真下から足中心までの傾き
			originalLegSlope := mmath.NewMQuaternionRotate(mmath.MVec3UnitYNeg, originalMorphLegCenterLocalPosition.Normalized())
			sizingLegSlope := mmath.NewMQuaternionRotate(mmath.MVec3UnitYNeg, sizingMorphLegCenterLocalPosition.Normalized())

			// 下半身根元から足までの真っ直ぐにしたときの長さ差
			originalLegCenterVerticalDiff := originalLegSlope.Inverted().MulVec3(originalMorphLegCenterLocalPosition).Truncate(1e-3)

			sizingLegCenterVerticalDiff := sizingLegSlope.Inverted().MulVec3(sizingMorphLegCenterLocalPosition).Truncate(1e-3)

			legCenterFromLowerScale := sizingLegCenterVerticalDiff.Dived(originalLegCenterVerticalDiff).Effective().One()

			// -------------------------

			originalLowerDelta := originalAllDeltas[index].Bones.GetByName(pmx.LOWER.String())
			originalLegCenterDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG_CENTER.String())

			// 下半身根元から見た足ボーンのローカル位置
			originalLegCenterLocalPosition := originalLowerDelta.FilledGlobalMatrix().Inverted().MulVec3(
				originalLegCenterDelta.FilledGlobalPosition())

			// 真っ直ぐにしたときのローカル位置
			originalLegCenterVerticalLocalPosition := originalLegSlope.Inverted().MulVec3(originalLegCenterLocalPosition).Truncate(1e-3)

			// スケール差を考慮した先の足ボーンのローカル位置
			sizingLegCenterVerticalLocalPosition := originalLegCenterVerticalLocalPosition.Muled(legCenterFromLowerScale)

			sizingLegCenterLocalPosition := sizingLegSlope.MulVec3(sizingLegCenterVerticalLocalPosition)

			sizingLowerDelta := sizingAllDeltas[index].Bones.GetByName(pmx.LOWER.String())
			sizingLegCenterIdealGlobalPosition := sizingLowerDelta.FilledGlobalMatrix().MulVec3(
				sizingLegCenterLocalPosition)

			if mlog.IsDebug() {
				originalLowerDelta := originalAllDeltas[index].Bones.GetByName(pmx.LOWER.String())
				originalLowerInitialPositions[index] = originalLowerDelta.FilledGlobalPosition()
				originalLowerInitialRotations[index] = originalLowerDelta.FilledFrameRotation()

				originalLegCenterInitialPositions[index] = originalLegCenterDelta.FilledGlobalPosition().Copy()
				originalLeftLegInitialPositions[index] = originalAllDeltas[index].Bones.GetByName(pmx.LEG_ROOT.Left()).FilledGlobalPosition().Copy()
				originalRightLegInitialPositions[index] = originalAllDeltas[index].Bones.GetByName(pmx.LEG_ROOT.Right()).FilledGlobalPosition().Copy()

				sizingLowerInitialPositions[index] = sizingLowerDelta.FilledGlobalPosition().Copy()
				sizingLegCenterInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(
					pmx.LEG_CENTER.String()).FilledGlobalPosition().Copy()
				sizingLeftLegInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(
					pmx.LEG.Left()).FilledGlobalPosition().Copy()
				sizingRightLegInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(
					pmx.LEG.Right()).FilledGlobalPosition().Copy()

				sizingLegCenterIdealPositions[index] = sizingLegCenterIdealGlobalPosition.Copy()

				sizingLowerDelta := sizingAllDeltas[index].Bones.GetByName(pmx.LOWER.String())
				sizingLowerInitialPositions[index] = sizingLowerDelta.FilledGlobalPosition().Copy()
				sizingLowerInitialRotations[index] = sizingLowerDelta.FilledFrameRotation()
			}

			// IK解決
			sizingLowerDeltas := deform.DeformIks(sizingSet.SizingConfigModel, sizingProcessMotion,
				sizingAllDeltas[index], float32(data),
				[]*pmx.Bone{lowerIkBone},
				[]*pmx.Bone{sizingSet.SizingLegCenterBone()},
				[]*mmath.MVec3{sizingLegCenterIdealGlobalPosition},
				trunk_lower_bone_names, 1, false, false)

			sizingLowerResultDelta := sizingLowerDeltas.Bones.GetByName(pmx.LOWER.String())
			sizingLowerResultRotations[index] = sizingLowerResultDelta.FilledFrameRotation()

			if mlog.IsDebug() {
				sizingLowerResultPositions[index] = sizingLowerResultDelta.FilledGlobalPosition().Copy()

				sizingLegCenterResultPositions[index] = sizingLowerDeltas.Bones.GetByName(pmx.LEG_CENTER.String()).FilledGlobalPosition().Copy()
				sizingLeftLegResultPositions[index] = sizingLowerDeltas.Bones.GetByName(pmx.LEG.Left()).FilledGlobalPosition().Copy()
				sizingRightLegResultPositions[index] = sizingLowerDeltas.Bones.GetByName(pmx.LEG.Right()).FilledGlobalPosition().Copy()
			}

			incrementCompletedCount()

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正03", sizingSet.Index, iterIndex, allCount)
		})
	if err != nil {
		return nil, err
	}

	if mlog.IsDebug() {
		motion := vmd.NewVmdMotion("")
		initialRotations := make([]*mmath.MQuaternion, len(allFrames))

		for i, iFrame := range allFrames {
			frame := float32(iFrame)

			for _, v := range [][]any{
				{originalLowerInitialPositions, originalLowerInitialRotations, "元今下半身"},
				{originalLegCenterInitialPositions, initialRotations, "元今足中心"},
				{originalLeftLegInitialPositions, initialRotations, "元今左足"},
				{originalRightLegInitialPositions, initialRotations, "元今右足"},
				{sizingLegCenterInitialPositions, initialRotations, "先今足中心"},
				{sizingLeftLegInitialPositions, initialRotations, "先今左足"},
				{sizingRightLegInitialPositions, initialRotations, "先今右足"},
				{sizingLegCenterIdealPositions, initialRotations, "先理足中心"},
				{sizingLowerResultPositions, sizingLowerResultRotations, "先結下半身"},
				{sizingLegCenterResultPositions, initialRotations, "先結足中心"},
				{sizingLeftLegResultPositions, initialRotations, "先結左足"},
				{sizingRightLegResultPositions, initialRotations, "先結右足"},
			} {
				positions := v[0].([]*mmath.MVec3)
				rotations := v[1].([]*mmath.MQuaternion)
				boneName := v[2].(string)
				bf := vmd.NewBoneFrame(frame)
				bf.Position = positions[i]
				bf.Rotation = rotations[i]
				motion.InsertBoneFrame(boneName, bf)
			}
		}

		outputVerboseMotion(verboseMotionName, sizingSet.OutputMotionPath, motion)
	}

	return sizingLowerResultRotations, nil
}

// updateLower は、補正した下半身回転をサイジング先モーションに反映します。
func (su *SizingLegUsecase) updateLower(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	lowerRotations []*mmath.MQuaternion, incrementCompletedCount func(),
) {
	for i, iFrame := range allFrames {
		frame := float32(iFrame)
		{
			bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingLowerBone().Name()).Get(frame)
			bf.Rotation = lowerRotations[i]
			sizingProcessMotion.InsertBoneFrame(sizingSet.SizingLowerBone().Name(), bf)
		}

		if i > 0 && i%1000 == 0 {
			processLog("足補正04", sizingSet.Index, i, len(allFrames))
		}

		incrementCompletedCount()
	}
}

func (su *SizingLegUsecase) createFullToeTailIkBone(sizingSet *domain.SizingSet, direction pmx.BoneDirection) (
	ikBone *pmx.Bone,
) {
	// legIkBone, _ := sizingSet.SizingConfigModel.Bones.GetLegIk(direction)
	legBone, _ := sizingSet.SizingConfigModel.Bones.GetLeg(direction)
	tailBone, _ := sizingSet.SizingConfigModel.Bones.GetToeTD(direction)
	ikBone = pmx.NewBoneByName(fmt.Sprintf("%s%sIk", pmx.MLIB_PREFIX, tailBone.Name()))

	ikBone.Position = tailBone.Position.Copy()
	ikBone.Ik = pmx.NewIk()
	ikBone.Ik.BoneIndex = tailBone.Index()
	ikBone.Ik.LoopCount = 10
	ikBone.Ik.UnitRotation = &mmath.MVec3{X: 1, Y: 0.0, Z: 0.0}
	ikBone.Ik.Links = make([]*pmx.IkLink, 0)

	for _, boneName := range []string{
		pmx.TOE_EX.StringFromDirection(direction),
		pmx.ANKLE_D.StringFromDirection(direction),
		pmx.KNEE_D.StringFromDirection(direction),
		pmx.LEG_D.StringFromDirection(direction),
		pmx.ANKLE.StringFromDirection(direction),
		pmx.KNEE.StringFromDirection(direction),
		pmx.LEG.StringFromDirection(direction),
	} {
		bone, _ := sizingSet.SizingConfigModel.Bones.GetByName(boneName)
		if bone == nil {
			continue
		}

		link := pmx.NewIkLink()
		link.BoneIndex = bone.Index()
		link.AngleLimit = true

		if bone.Index() == legBone.Index() {
			// 足ボーンだけ動かす
			link.AngleLimit = false
		}

		ikBone.Ik.Links = append(ikBone.Ik.Links, link)

		if bone.Index() == legBone.Index() {
			// 足までいったら終了
			break
		}
	}

	return ikBone
}

// calculateAdjustedLegIK は、足IK 補正の計算を並列処理で行い、各フレームごとの位置・回転補正値を算出します。
func (su *SizingLegUsecase) calculateAdjustedLegIK(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int,
	originalAllDeltas, sizingAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas []*delta.VmdDeltas,
	sizingProcessMotion *vmd.VmdMotion, incrementCompletedCount func(), verboseMotionKey string,
) (legIkPositions [][]*mmath.MVec3, legRotations [][]*mmath.MQuaternion, err error) {
	legIkPositions = make([][]*mmath.MVec3, 2)
	legRotations = make([][]*mmath.MQuaternion, 2)

	originalLegInitialPositions := make([][]*mmath.MVec3, 2)
	originalKneeInitialPositions := make([][]*mmath.MVec3, 2)
	originalAnkleInitialPositions := make([][]*mmath.MVec3, 2)
	originalKneeDInitialPositions := make([][]*mmath.MVec3, 2)
	originalAnkleDInitialPositions := make([][]*mmath.MVec3, 2)
	originalHeelDInitialPositions := make([][]*mmath.MVec3, 2)
	originalToeTailDInitialPositions := make([][]*mmath.MVec3, 2)

	sizingLegInitialPositions := make([][]*mmath.MVec3, 2)
	sizingKneeInitialPositions := make([][]*mmath.MVec3, 2)
	sizingAnkleInitialPositions := make([][]*mmath.MVec3, 2)
	sizingKneeDInitialPositions := make([][]*mmath.MVec3, 2)
	sizingAnkleDInitialPositions := make([][]*mmath.MVec3, 2)
	sizingHeelDInitialPositions := make([][]*mmath.MVec3, 2)
	sizingToeTailDInitialPositions := make([][]*mmath.MVec3, 2)

	sizingToeTailDIdealPositions := make([][]*mmath.MVec3, 2)

	sizingLegResultPositions := make([][]*mmath.MVec3, 2)
	sizingKneeResultPositions := make([][]*mmath.MVec3, 2)
	sizingAnkleResultPositions := make([][]*mmath.MVec3, 2)
	sizingKneeDResultPositions := make([][]*mmath.MVec3, 2)
	sizingAnkleDResultPositions := make([][]*mmath.MVec3, 2)
	sizingToeTailDResultPositions := make([][]*mmath.MVec3, 2)
	sizingHeelDResultPositions := make([][]*mmath.MVec3, 2)

	for i := range directions {
		legIkPositions[i] = make([]*mmath.MVec3, len(allFrames))
		legRotations[i] = make([]*mmath.MQuaternion, len(allFrames))

		originalLegInitialPositions[i] = make([]*mmath.MVec3, len(allFrames))
		originalKneeInitialPositions[i] = make([]*mmath.MVec3, len(allFrames))
		originalAnkleInitialPositions[i] = make([]*mmath.MVec3, len(allFrames))
		originalKneeDInitialPositions[i] = make([]*mmath.MVec3, len(allFrames))
		originalAnkleDInitialPositions[i] = make([]*mmath.MVec3, len(allFrames))
		originalHeelDInitialPositions[i] = make([]*mmath.MVec3, len(allFrames))
		originalToeTailDInitialPositions[i] = make([]*mmath.MVec3, len(allFrames))

		sizingLegInitialPositions[i] = make([]*mmath.MVec3, len(allFrames))
		sizingKneeInitialPositions[i] = make([]*mmath.MVec3, len(allFrames))
		sizingAnkleInitialPositions[i] = make([]*mmath.MVec3, len(allFrames))
		sizingKneeDInitialPositions[i] = make([]*mmath.MVec3, len(allFrames))
		sizingAnkleDInitialPositions[i] = make([]*mmath.MVec3, len(allFrames))
		sizingHeelDInitialPositions[i] = make([]*mmath.MVec3, len(allFrames))
		sizingToeTailDInitialPositions[i] = make([]*mmath.MVec3, len(allFrames))

		sizingToeTailDIdealPositions[i] = make([]*mmath.MVec3, len(allFrames))

		sizingLegResultPositions[i] = make([]*mmath.MVec3, len(allFrames))
		sizingKneeResultPositions[i] = make([]*mmath.MVec3, len(allFrames))
		sizingAnkleResultPositions[i] = make([]*mmath.MVec3, len(allFrames))
		sizingKneeDResultPositions[i] = make([]*mmath.MVec3, len(allFrames))
		sizingAnkleDResultPositions[i] = make([]*mmath.MVec3, len(allFrames))
		sizingToeTailDResultPositions[i] = make([]*mmath.MVec3, len(allFrames))
		sizingHeelDResultPositions[i] = make([]*mmath.MVec3, len(allFrames))
	}

	toeTailDBones := make([]*pmx.Bone, 2)
	toeTailDBones[0] = sizingSet.SizingLeftToeTailDBone()
	toeTailDBones[1] = sizingSet.SizingRightToeTailDBone()

	toeTailDIkBones := make([]*pmx.Bone, 2)
	toeTailDIkBones[0] = su.createFullToeTailIkBone(sizingSet, pmx.BONE_DIRECTION_LEFT)
	toeTailDIkBones[1] = su.createFullToeTailIkBone(sizingSet, pmx.BONE_DIRECTION_RIGHT)

	legIkBones := make([]*pmx.Bone, 2)
	legIkBones[0] = sizingSet.SizingLeftLegIkBone()
	legIkBones[1] = sizingSet.SizingRightLegIkBone()

	legIkParentBones := make([]*pmx.Bone, 2)
	legIkParentBones[0] = sizingSet.SizingLeftLegIkBone().ParentBone
	legIkParentBones[1] = sizingSet.SizingRightLegIkBone().ParentBone

	err = miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			for d, direction := range directions {
				// 足根元から足首地面までの長さ差
				originalMorphLegDelta := originalMorphAllDeltas[index].Bones.GetByName(
					pmx.LEG.StringFromDirection(direction))
				originalMorphKneeDelta := originalMorphAllDeltas[index].Bones.GetByName(
					pmx.KNEE.StringFromDirection(direction))
				originalMorphAnkleDelta := originalMorphAllDeltas[index].Bones.GetByName(
					pmx.ANKLE.StringFromDirection(direction))
				sizingMorphLegDelta := sizingMorphAllDeltas[index].Bones.GetByName(
					pmx.LEG.StringFromDirection(direction))
				sizingMorphKneeDelta := sizingMorphAllDeltas[index].Bones.GetByName(
					pmx.KNEE.StringFromDirection(direction))
				sizingMorphAnkleDelta := sizingMorphAllDeltas[index].Bones.GetByName(
					pmx.ANKLE.StringFromDirection(direction))

				originalAnkleGroundPosition := originalMorphAnkleDelta.FilledGlobalPosition().Copy()
				originalAnkleGroundPosition.Y = 0
				sizingAnkleGroundPosition := sizingMorphAnkleDelta.FilledGlobalPosition().Copy()
				sizingAnkleGroundPosition.Y = 0

				originalLegLength := originalMorphLegDelta.FilledGlobalPosition().Distance(
					originalMorphKneeDelta.FilledGlobalPosition()) +
					originalMorphKneeDelta.FilledGlobalPosition().Distance(originalAnkleGroundPosition)
				sizingLegLength := sizingMorphLegDelta.FilledGlobalPosition().Distance(
					sizingMorphKneeDelta.FilledGlobalPosition()) +
					sizingMorphKneeDelta.FilledGlobalPosition().Distance(sizingAnkleGroundPosition)

				legScale := sizingLegLength / originalLegLength

				// -------------------------------

				originalLegDelta := originalAllDeltas[index].Bones.GetByName(
					pmx.LEG.StringFromDirection(direction))
				originalToeTailDDelta := originalAllDeltas[index].Bones.GetByName(
					pmx.TOE_T_D.StringFromDirection(direction))

				sizingLegDelta := sizingAllDeltas[index].Bones.GetByName(
					pmx.LEG.StringFromDirection(direction))

				// 足根元ボーンから見たつま先先Dまでの相対位置（角度を見たくないのでグローバルの差分で）
				originalToeTailDRelativePosition := originalToeTailDDelta.FilledGlobalPosition().Subed(
					originalLegDelta.FilledGlobalPosition())

				// スケール差を考慮した先のつま先Dボーンのローカル位置
				sizingToeTailDRelativePosition := originalToeTailDRelativePosition.MuledScalar(legScale)

				toeTailDIdealPosition := sizingLegDelta.FilledGlobalPosition().Added(sizingToeTailDRelativePosition)

				if mlog.IsDebug() {
					sizingToeTailDIdealPositions[d][index] = toeTailDIdealPosition.Copy()

					originalLegInitialPositions[d][index] = originalAllDeltas[index].Bones.GetByName(
						pmx.LEG.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					originalKneeInitialPositions[d][index] = originalAllDeltas[index].Bones.GetByName(
						pmx.KNEE.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					originalAnkleInitialPositions[d][index] = originalAllDeltas[index].Bones.GetByName(
						pmx.ANKLE.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					originalKneeDInitialPositions[d][index] = originalAllDeltas[index].Bones.GetByName(
						pmx.KNEE_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					originalAnkleDInitialPositions[d][index] = originalAllDeltas[index].Bones.GetByName(
						pmx.ANKLE_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					originalHeelDInitialPositions[d][index] = originalAllDeltas[index].Bones.GetByName(
						pmx.HEEL_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					originalToeTailDInitialPositions[d][index] = originalAllDeltas[index].Bones.GetByName(
						pmx.TOE_T_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()

					sizingLegInitialPositions[d][index] = sizingAllDeltas[index].Bones.GetByName(
						pmx.LEG.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingKneeInitialPositions[d][index] = sizingAllDeltas[index].Bones.GetByName(
						pmx.KNEE.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingAnkleInitialPositions[d][index] = sizingAllDeltas[index].Bones.GetByName(
						pmx.ANKLE.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingKneeDInitialPositions[d][index] = sizingAllDeltas[index].Bones.GetByName(
						pmx.KNEE_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingAnkleDInitialPositions[d][index] = sizingAllDeltas[index].Bones.GetByName(
						pmx.ANKLE_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingHeelDInitialPositions[d][index] = sizingAllDeltas[index].Bones.GetByName(
						pmx.HEEL_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingToeTailDInitialPositions[d][index] = sizingAllDeltas[index].Bones.GetByName(
						pmx.TOE_T_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
				}

				// IK解決
				sizingLegDeltas := deform.DeformIks(sizingSet.SizingConfigModel, sizingProcessMotion,
					sizingAllDeltas[index], float32(data),
					[]*pmx.Bone{toeTailDIkBones[d]},
					[]*pmx.Bone{toeTailDBones[d]},
					[]*mmath.MVec3{toeTailDIdealPosition},
					leg_direction_bone_names[d], 1, false, false)

				if mlog.IsDebug() {
					sizingLegResultPositions[d][index] = sizingLegDeltas.Bones.GetByName(
						pmx.LEG.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingKneeResultPositions[d][index] = sizingLegDeltas.Bones.GetByName(
						pmx.KNEE.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingAnkleResultPositions[d][index] = sizingLegDeltas.Bones.GetByName(
						pmx.ANKLE.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingKneeDResultPositions[d][index] = sizingLegDeltas.Bones.GetByName(
						pmx.KNEE_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingAnkleDResultPositions[d][index] = sizingLegDeltas.Bones.GetByName(
						pmx.ANKLE_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingHeelDResultPositions[d][index] = sizingLegDeltas.Bones.GetByName(
						pmx.HEEL_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingToeTailDResultPositions[d][index] = sizingLegDeltas.Bones.GetByName(
						pmx.TOE_T_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
				}

				ankleDelta := sizingLegDeltas.Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))

				// 足IKの親からみた足IKデフォルト位置からみた現在の足首のローカル位置
				sizingLegIkParentDelta := sizingLegDeltas.Bones.Get(legIkParentBones[d].Index())
				sizingLegIkMatrix := sizingLegIkParentDelta.FilledGlobalMatrix().Translated(
					legIkBones[d].ParentRelativePosition)
				legIkPositions[d][index] = sizingLegIkMatrix.Inverted().MulVec3(ankleDelta.FilledGlobalPosition())

				legDelta := sizingLegDeltas.Bones.GetByName(pmx.LEG.StringFromDirection(direction))
				legRotations[d][index] = legDelta.FilledFrameRotation().Copy()
			}

			incrementCompletedCount()

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正07", sizingSet.Index, iterIndex, allCount)
		})
	if err != nil {
		return nil, nil, err
	}

	if mlog.IsDebug() {
		motion := vmd.NewVmdMotion("")
		initialRotations := make([][]*mmath.MQuaternion, 2)
		initialRotations[0] = make([]*mmath.MQuaternion, len(allFrames))
		initialRotations[1] = make([]*mmath.MQuaternion, len(allFrames))

		for i, iFrame := range allFrames {
			frame := float32(iFrame)

			for _, v := range [][]any{
				{legIkPositions, legRotations, "先結%s足IK1"},
				{originalLegInitialPositions, initialRotations, "元今%s足1"},
				{originalKneeInitialPositions, initialRotations, "元今%s膝1"},
				{originalAnkleInitialPositions, initialRotations, "元今%s足首1"},
				{originalKneeDInitialPositions, initialRotations, "元今%s膝D1"},
				{originalAnkleDInitialPositions, initialRotations, "元今%s足首D1"},
				{originalHeelDInitialPositions, initialRotations, "元今%s踵D1"},
				{originalToeTailDInitialPositions, initialRotations, "元今%s先D1"},
				{sizingLegInitialPositions, initialRotations, "先今%s足1"},
				{sizingKneeInitialPositions, initialRotations, "先今%s膝1"},
				{sizingAnkleInitialPositions, initialRotations, "先今%s足首1"},
				{sizingKneeDInitialPositions, initialRotations, "先今%s膝D1"},
				{sizingAnkleDInitialPositions, initialRotations, "先今%s足首D1"},
				{sizingHeelDInitialPositions, initialRotations, "先今%s踵D1"},
				{sizingToeTailDInitialPositions, initialRotations, "先今%s先D1"},
				{sizingToeTailDIdealPositions, initialRotations, "先理%s先D1"},
				{sizingLegResultPositions, initialRotations, "先結%s足1"},
				{sizingKneeResultPositions, initialRotations, "先結%s膝1"},
				{sizingAnkleResultPositions, initialRotations, "先結%s足首1"},
				{sizingKneeDResultPositions, initialRotations, "先結%s膝D1"},
				{sizingAnkleDResultPositions, initialRotations, "先結%s足首D1"},
				{sizingToeTailDResultPositions, initialRotations, "先結%s先D1"},
				{sizingHeelDResultPositions, initialRotations, "先結%s踵D1"},
			} {
				positions := v[0].([][]*mmath.MVec3)
				rotations := v[1].([][]*mmath.MQuaternion)
				for d, direction := range directions {
					boneName := fmt.Sprintf(v[2].(string), direction.String())
					bf := vmd.NewBoneFrame(frame)
					bf.Position = positions[d][i]
					bf.Rotation = rotations[d][i]
					motion.InsertBoneFrame(boneName, bf)
				}
			}
		}

		outputVerboseMotion(verboseMotionKey, sizingSet.OutputMotionPath, motion)
	}

	return legIkPositions, legRotations, nil
}

// updateLegIkAndFk は、計算済みの足IK 補正位置と回転値をモーションデータに反映させます。
func (su *SizingLegUsecase) updateLegIkAndFk(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	legIkPositions [][]*mmath.MVec3, legRotations [][]*mmath.MQuaternion, incrementCompletedCount func(),
) {
	for i, iFrame := range allFrames {
		frame := float32(iFrame)

		for d, direction := range directions {
			{
				bf := sizingProcessMotion.BoneFrames.Get(pmx.LEG_IK.StringFromDirection(direction)).Get(frame)
				bf.Position = legIkPositions[d][i]
				sizingProcessMotion.InsertBoneFrame(pmx.LEG_IK.StringFromDirection(direction), bf)
			}
			{
				bf := sizingProcessMotion.BoneFrames.Get(pmx.LEG.StringFromDirection(direction)).Get(frame)
				bf.Rotation = legRotations[d][i]
				sizingProcessMotion.InsertBoneFrame(pmx.LEG.StringFromDirection(direction), bf)
			}
		}

		if i > 0 && i%1000 == 0 {
			processLog("足補正08", sizingSet.Index, i, len(allFrames))
		}

		incrementCompletedCount()
	}
}

// calculateAdjustedCenter は、センターおよびグルーブの位置補正を並列処理で計算します。
func (su *SizingLegUsecase) calculateAdjustedCenter(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, moveScale *mmath.MVec3,
	originalAllDeltas, sizingAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas []*delta.VmdDeltas,
	sizingProcessMotion *vmd.VmdMotion, incrementCompletedCount func(), verboseMotionName string,
) (
	rootPositions, centerPositions, groovePositions, leftLegParentPositions, rightLegParentPositions []*mmath.MVec3,
	err error,
) {
	centerPositions = make([]*mmath.MVec3, len(allFrames))
	groovePositions = make([]*mmath.MVec3, len(allFrames))
	rootPositions = make([]*mmath.MVec3, len(allFrames))
	leftLegParentPositions = make([]*mmath.MVec3, len(allFrames))
	rightLegParentPositions = make([]*mmath.MVec3, len(allFrames))

	originalBodyAxisPositions := make([]*mmath.MVec3, len(allFrames))
	sizingBodyAxisPositions := make([]*mmath.MVec3, len(allFrames))
	sizingBodyAxisIdealPositions := make([]*mmath.MVec3, len(allFrames))

	isActiveGroove := false
	sizingProcessMotion.BoneFrames.Get(pmx.GROOVE.String()).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
		if !mmath.NearEquals(bf.FilledPosition().Y, 0.0, 1e-3) {
			isActiveGroove = true
			return false
		}
		return true
	})

	err = miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			// 体軸までのYの長さ
			originalMorphBodyAxisDelta := originalMorphAllDeltas[index].Bones.GetByName(pmx.BODY_AXIS.String())
			sizingMorphBodyAxisDelta := sizingMorphAllDeltas[index].Bones.GetByName(pmx.BODY_AXIS.String())
			originalInitialBodyAxisY := originalMorphBodyAxisDelta.FilledGlobalPosition().Y
			sizingInitialBodyAxisY := sizingMorphBodyAxisDelta.FilledGlobalPosition().Y

			// 先の初期姿勢におけるセンターの親から見た体軸のローカル位置
			sizingMorphCenterParentDelta := sizingMorphAllDeltas[index].Bones.GetByName(
				sizingSet.SizingCenterBone().ParentBone.Name())

			sizingBodyAxisInitialLocalPosition := sizingMorphCenterParentDelta.FilledGlobalMatrix().Inverted().MulVec3(
				sizingMorphBodyAxisDelta.FilledGlobalPosition())

			// --------------------------------

			// 元の体軸
			originalBodyAxisDelta := originalAllDeltas[index].Bones.GetByName(pmx.BODY_AXIS.String())
			// 元のセンター親
			originalCenterParentDelta := originalAllDeltas[index].Bones.GetByName(
				sizingSet.OriginalCenterBone().ParentBone.Name())
			// センターの親から見た元の体軸のローカル位置
			originalBodyAxisLocalPosition := originalCenterParentDelta.FilledGlobalMatrix().Inverted().MulVec3(originalBodyAxisDelta.FilledGlobalPosition())

			// 体軸のローカル位置の高さが、元の体軸の高さに対してどのくらいの比率か
			bodyAxisYRatio := originalBodyAxisLocalPosition.Y / originalInitialBodyAxisY

			// 先の体軸の高さに、その比率をかける
			sizingBodyAxisY := sizingInitialBodyAxisY * bodyAxisYRatio

			// 先の体軸
			sizingBodyAxisDelta := sizingAllDeltas[index].Bones.GetByName(pmx.BODY_AXIS.String())
			// 先のセンター親
			sizingCenterParentDelta := sizingAllDeltas[index].Bones.GetByName(
				sizingSet.SizingCenterBone().ParentBone.Name())

			// 先の理想体軸
			sizingBodyAxisIdealLocalPosition := originalBodyAxisLocalPosition.Muled(moveScale)
			// ローカル位置のYを置き換え
			sizingBodyAxisIdealLocalPosition.Y = sizingBodyAxisY

			// 初期姿勢からの差分
			sizingLegCenterDiff := sizingBodyAxisIdealLocalPosition.Subed(sizingBodyAxisInitialLocalPosition)

			centerPositions[index] = sizingLegCenterDiff.Copy()

			if isActiveGroove {
				groovePositions[index] = &mmath.MVec3{X: 0.0, Y: sizingLegCenterDiff.Y, Z: 0.0}
				centerPositions[index].Y = 0.0
			}

			// 全親・足IK親は単純なスケール
			originalRootDelta := originalAllDeltas[index].Bones.GetByName(pmx.ROOT.String())
			rootPositions[index] = originalRootDelta.FilledGlobalPosition().Muled(moveScale)

			leftLegParentDelta := originalAllDeltas[index].Bones.GetByName(
				sizingSet.OriginalLeftLegIkParentBone().Name())
			leftLegParentParentDelta := originalAllDeltas[index].Bones.GetByName(
				sizingSet.OriginalLeftLegIkParentBone().ParentBone.Name())
			leftLegParentLocalPosition := leftLegParentParentDelta.FilledGlobalMatrix().Inverted().MulVec3(leftLegParentDelta.FilledGlobalPosition())
			leftLegParentPositions[index] = leftLegParentLocalPosition.Muled(moveScale)

			rightLegParentDelta := originalAllDeltas[index].Bones.GetByName(
				sizingSet.OriginalRightLegIkParentBone().Name())
			rightLegParentParentDelta := originalAllDeltas[index].Bones.GetByName(
				sizingSet.OriginalRightLegIkParentBone().ParentBone.Name())
			rightLegParentLocalPosition := rightLegParentParentDelta.FilledGlobalMatrix().Inverted().MulVec3(rightLegParentDelta.FilledGlobalPosition())
			rightLegParentPositions[index] = rightLegParentLocalPosition.Muled(moveScale)

			if mlog.IsDebug() {
				originalBodyAxisPositions[index] = originalBodyAxisDelta.FilledGlobalPosition()
				sizingBodyAxisPositions[index] = sizingBodyAxisDelta.FilledGlobalPosition()
				sizingBodyAxisIdealPositions[index] = sizingCenterParentDelta.FilledGlobalMatrix().MulVec3(sizingBodyAxisIdealLocalPosition)
			}

			incrementCompletedCount()

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正05", sizingSet.Index, iterIndex, allCount)
		})
	if err != nil {
		return nil, nil, nil, nil, nil, err
	}

	if mlog.IsDebug() {
		motion := vmd.NewVmdMotion("")

		for i, iFrame := range allFrames {
			frame := float32(iFrame)
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = originalBodyAxisPositions[i]
				motion.InsertBoneFrame("元体軸", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = sizingBodyAxisPositions[i]
				motion.InsertBoneFrame("先体軸", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = sizingBodyAxisIdealPositions[i]
				motion.InsertBoneFrame("先理想体軸", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				if groovePositions[i] == nil {
					bf.Position = centerPositions[i]
				} else {
					bf.Position = centerPositions[i].Added(groovePositions[i])
				}
				motion.InsertBoneFrame("先センター", bf)
			}
		}

		outputVerboseMotion(verboseMotionName, sizingSet.OutputMotionPath, motion)
	}

	return rootPositions, centerPositions, groovePositions, leftLegParentPositions, rightLegParentPositions, nil
}

// updateCenter は、計算したセンター・グルーブ位置をサイジング先モーションに反映します。
func (su *SizingLegUsecase) updateCenter(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	rootPositions, centerPositions, groovePositions, leftLegParentPositions, rightLegParentPositions []*mmath.MVec3,
	incrementCompletedCount func(),
) {
	for i, iFrame := range allFrames {
		frame := float32(iFrame)

		for _, v := range [][]any{
			{pmx.ROOT.String(), rootPositions[i]},
			{pmx.CENTER.String(), centerPositions[i]},
			{pmx.GROOVE.String(), groovePositions[i]},
			{pmx.LEG_IK_PARENT.Left(), leftLegParentPositions[i]},
			{pmx.LEG_IK_PARENT.Right(), rightLegParentPositions[i]},
		} {
			boneName := v[0].(string)
			position := v[1].(*mmath.MVec3)

			bf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
			bf.Position = position
			sizingProcessMotion.InsertBoneFrame(boneName, bf)
		}

		if i > 0 && i%1000 == 0 {
			processLog("足補正06", sizingSet.Index, i, len(allFrames))
		}

		incrementCompletedCount()
	}
}

func (su *SizingLegUsecase) createFullAnkleIkBone(sizingSet *domain.SizingSet, direction pmx.BoneDirection) (
	ikBone *pmx.Bone,
) {
	legIkBone, _ := sizingSet.SizingConfigModel.Bones.GetLegIk(direction)
	legBone, _ := sizingSet.SizingConfigModel.Bones.GetLeg(direction)
	tailBone, _ := sizingSet.SizingConfigModel.Bones.GetAnkleD(direction)
	ikBone = pmx.NewBoneByName(fmt.Sprintf("%s%sIk", pmx.MLIB_PREFIX, tailBone.Name()))

	ikBone.Position = tailBone.Position.Copy()
	ikBone.Ik = pmx.NewIk()
	ikBone.Ik.BoneIndex = tailBone.Index()
	ikBone.Ik.LoopCount = 100
	ikBone.Ik.UnitRotation = &mmath.MVec3{X: 0.1, Y: 0.0, Z: 0.0}
	ikBone.Ik.Links = make([]*pmx.IkLink, 0)

	for _, boneName := range []string{
		pmx.KNEE_D.StringFromDirection(direction),
		pmx.LEG_D.StringFromDirection(direction),
		pmx.KNEE.StringFromDirection(direction),
		pmx.LEG.StringFromDirection(direction),
	} {
		bone, _ := sizingSet.SizingConfigModel.Bones.GetByName(boneName)
		if bone == nil {
			continue
		}

		link := pmx.NewIkLink()
		link.BoneIndex = bone.Index()
		link.AngleLimit = true

		for _, l := range legIkBone.Ik.Links {
			if l.BoneIndex == link.BoneIndex {
				// リンクの設定は足IKに準拠
				link.AngleLimit = l.AngleLimit
				link.MinAngleLimit = l.MinAngleLimit
				link.MaxAngleLimit = l.MaxAngleLimit
			}
		}

		ikBone.Ik.Links = append(ikBone.Ik.Links, link)

		if bone.Index() == legBone.Index() {
			// 足までいったら終了
			break
		}
	}

	return ikBone
}

// calculateAdjustedLegIK は、足IK 補正の計算を並列処理で行い、各フレームごとの位置・回転補正値を算出します。
func (su *SizingLegUsecase) calculateAdjustedLegIK2(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int,
	originalAllDeltas, sizingAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas []*delta.VmdDeltas,
	sizingProcessMotion *vmd.VmdMotion, incrementCompletedCount func(), verboseMotionKey string,
) (legIkPositions [][]*mmath.MVec3,
	legIkRotations, legRotations, kneeRotations, ankleRotations [][]*mmath.MQuaternion, err error) {

	legIkPositions = make([][]*mmath.MVec3, 2)
	legIkRotations = make([][]*mmath.MQuaternion, 2)
	legRotations = make([][]*mmath.MQuaternion, 2)
	kneeRotations = make([][]*mmath.MQuaternion, 2)
	ankleRotations = make([][]*mmath.MQuaternion, 2)

	sizingLegInitialPositions := make([][]*mmath.MVec3, 2)
	sizingKneeInitialPositions := make([][]*mmath.MVec3, 2)
	sizingAnkleInitialPositions := make([][]*mmath.MVec3, 2)
	sizingKneeDInitialPositions := make([][]*mmath.MVec3, 2)
	sizingAnkleDInitialPositions := make([][]*mmath.MVec3, 2)
	sizingHeelDInitialPositions := make([][]*mmath.MVec3, 2)
	sizingToeTailDInitialPositions := make([][]*mmath.MVec3, 2)

	sizingLegResultPositions := make([][]*mmath.MVec3, 2)
	sizingKneeResultPositions := make([][]*mmath.MVec3, 2)
	sizingAnkleResultPositions := make([][]*mmath.MVec3, 2)
	sizingKneeDResultPositions := make([][]*mmath.MVec3, 2)
	sizingAnkleDResultPositions := make([][]*mmath.MVec3, 2)
	sizingToeTailDResultPositions := make([][]*mmath.MVec3, 2)
	sizingHeelDResultPositions := make([][]*mmath.MVec3, 2)

	sizingAnkleIdealPositions := make([][]*mmath.MVec3, 2)
	sizingToeTargetIdealPositions := make([][]*mmath.MVec3, 2)

	for d := range directions {
		legIkPositions[d] = make([]*mmath.MVec3, len(allFrames))
		legIkRotations[d] = make([]*mmath.MQuaternion, len(allFrames))
		legRotations[d] = make([]*mmath.MQuaternion, len(allFrames))
		kneeRotations[d] = make([]*mmath.MQuaternion, len(allFrames))
		ankleRotations[d] = make([]*mmath.MQuaternion, len(allFrames))

		sizingLegInitialPositions[d] = make([]*mmath.MVec3, len(allFrames))
		sizingKneeInitialPositions[d] = make([]*mmath.MVec3, len(allFrames))
		sizingAnkleInitialPositions[d] = make([]*mmath.MVec3, len(allFrames))
		sizingKneeDInitialPositions[d] = make([]*mmath.MVec3, len(allFrames))
		sizingAnkleDInitialPositions[d] = make([]*mmath.MVec3, len(allFrames))
		sizingHeelDInitialPositions[d] = make([]*mmath.MVec3, len(allFrames))
		sizingToeTailDInitialPositions[d] = make([]*mmath.MVec3, len(allFrames))

		sizingLegResultPositions[d] = make([]*mmath.MVec3, len(allFrames))
		sizingKneeResultPositions[d] = make([]*mmath.MVec3, len(allFrames))
		sizingAnkleResultPositions[d] = make([]*mmath.MVec3, len(allFrames))
		sizingKneeDResultPositions[d] = make([]*mmath.MVec3, len(allFrames))
		sizingAnkleDResultPositions[d] = make([]*mmath.MVec3, len(allFrames))
		sizingToeTailDResultPositions[d] = make([]*mmath.MVec3, len(allFrames))
		sizingHeelDResultPositions[d] = make([]*mmath.MVec3, len(allFrames))

		sizingAnkleIdealPositions[d] = make([]*mmath.MVec3, len(allFrames))
		sizingToeTargetIdealPositions[d] = make([]*mmath.MVec3, len(allFrames))
	}

	ankleBones := make([]*pmx.Bone, 2)
	ankleBones[0] = sizingSet.SizingLeftAnkleBone()
	ankleBones[1] = sizingSet.SizingRightAnkleBone()

	toeIkBones := make([]*pmx.Bone, 2)
	toeIkBones[0] = sizingSet.SizingLeftToeIkBone()
	toeIkBones[1] = sizingSet.SizingRightToeIkBone()

	toeIkTargetBones := make([]*pmx.Bone, 2)
	toeIkTargetBones[0], _ = sizingSet.SizingConfigModel.Bones.Get(toeIkBones[0].Ik.BoneIndex)
	toeIkTargetBones[1], _ = sizingSet.SizingConfigModel.Bones.Get(toeIkBones[1].Ik.BoneIndex)

	originalAnkleBones := make([]*pmx.Bone, 2)
	originalAnkleBones[0] = sizingSet.OriginalLeftAnkleBone()
	originalAnkleBones[1] = sizingSet.OriginalRightAnkleBone()

	ankleIkBones := make([]*pmx.Bone, 2)
	ankleIkBones[0] = su.createFullAnkleIkBone(sizingSet, pmx.BONE_DIRECTION_LEFT)
	ankleIkBones[1] = su.createFullAnkleIkBone(sizingSet, pmx.BONE_DIRECTION_RIGHT)

	sizingLegIkBones := make([]*pmx.Bone, 2)
	sizingLegIkBones[0] = sizingSet.SizingLeftLegIkBone()
	sizingLegIkBones[1] = sizingSet.SizingRightLegIkBone()

	sizingLegIkParentBones := make([]*pmx.Bone, 2)
	sizingLegIkParentBones[0] = sizingSet.SizingLeftLegIkBone().ParentBone
	sizingLegIkParentBones[1] = sizingSet.SizingRightLegIkBone().ParentBone

	err = miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			for d, direction := range directions {
				// 足根元から足首地面までの長さ差
				originalAnkleLength := originalMorphAllDeltas[index].Bones.GetByName(
					pmx.ANKLE.StringFromDirection(direction)).FilledGlobalPosition().Y
				sizingAnkleLength := sizingMorphAllDeltas[index].Bones.GetByName(
					pmx.ANKLE.StringFromDirection(direction)).FilledGlobalPosition().Y
				ankleScale := sizingAnkleLength / originalAnkleLength

				// --------------------------------

				ankleDelta := sizingAllDeltas[index].Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))

				ankleIdealGlobalPosition := ankleDelta.FilledGlobalPosition().Copy()
				ankleIdealGlobalPosition.Y += su.calculateAnkleYDiff(originalAllDeltas[index],
					sizingAllDeltas[index], originalAnkleBones[d], direction, ankleScale)
				ankleIdealGlobalPosition.Z += 0.1

				toeTargetDelta := sizingAllDeltas[index].Bones.Get(toeIkTargetBones[d].Index())

				ankleYDiff := ankleIdealGlobalPosition.Y - ankleDelta.FilledGlobalPosition().Y
				toeTargetGlobalPosition := toeTargetDelta.FilledGlobalPosition().Copy()
				toeTargetGlobalPosition.Y += ankleYDiff
				toeTargetGlobalPosition.Z += 0.1 // あえて少しズラして、MMD上で再計算されるようにする

				if mlog.IsDebug() {
					sizingAnkleIdealPositions[d][index] = ankleIdealGlobalPosition.Copy()
					sizingToeTargetIdealPositions[d][index] = toeTargetGlobalPosition.Copy()

					sizingLegInitialPositions[d][index] = sizingAllDeltas[index].Bones.GetByName(
						pmx.LEG.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingKneeInitialPositions[d][index] = sizingAllDeltas[index].Bones.GetByName(
						pmx.KNEE.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingAnkleInitialPositions[d][index] = sizingAllDeltas[index].Bones.GetByName(
						pmx.ANKLE.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingKneeDInitialPositions[d][index] = sizingAllDeltas[index].Bones.GetByName(
						pmx.KNEE_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingAnkleDInitialPositions[d][index] = sizingAllDeltas[index].Bones.GetByName(
						pmx.ANKLE_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingHeelDInitialPositions[d][index] = sizingAllDeltas[index].Bones.GetByName(
						pmx.HEEL_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingToeTailDInitialPositions[d][index] = sizingAllDeltas[index].Bones.GetByName(
						pmx.TOE_T_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
				}

				// IK解決
				sizingLegDeltas := deform.DeformIks(sizingSet.SizingConfigModel, sizingProcessMotion,
					sizingAllDeltas[index], float32(data),
					[]*pmx.Bone{ankleIkBones[d], toeIkBones[d]},
					[]*pmx.Bone{ankleBones[d], toeIkTargetBones[d]},
					[]*mmath.MVec3{ankleIdealGlobalPosition, toeTargetGlobalPosition},
					leg_direction_bone_names[d], 1, false, false)

				// 足IKの親からみた足IKデフォルト位置からみた現在の足首のローカル位置
				sizingLegIkMorphDelta := sizingMorphAllDeltas[index].Bones.Get(sizingLegIkBones[d].Index())
				sizingLegIkParentMorphDelta := sizingMorphAllDeltas[index].Bones.Get(sizingLegIkParentBones[d].Index())
				sizingLegIkMorphLocalPosition := sizingLegIkParentMorphDelta.FilledGlobalMatrix().Inverted().MulVec3(
					sizingLegIkMorphDelta.FilledGlobalPosition())

				sizingLegIkParentDelta := sizingLegDeltas.Bones.Get(sizingLegIkParentBones[d].Index())
				sizingLegIkMatrix := sizingLegIkParentDelta.FilledGlobalMatrix().Translated(sizingLegIkMorphLocalPosition)
				legIkPositions[d][index] = sizingLegIkMatrix.Inverted().MulVec3(ankleIdealGlobalPosition)
				legIkRotations[d][index] = sizingLegDeltas.Bones.GetByName(pmx.LEG_IK.StringFromDirection(direction)).FilledFrameRotation()

				legRotations[d][index] = sizingLegDeltas.Bones.GetByName(pmx.LEG.StringFromDirection(direction)).FilledFrameRotation().Copy()
				kneeRotations[d][index] = sizingLegDeltas.Bones.GetByName(pmx.KNEE.StringFromDirection(direction)).FilledFrameRotation().Copy()
				ankleRotations[d][index] = sizingLegDeltas.Bones.GetByName(pmx.ANKLE.StringFromDirection(direction)).FilledFrameRotation().Copy()

				if mlog.IsDebug() {
					sizingLegResultPositions[d][index] = sizingLegDeltas.Bones.GetByName(
						pmx.LEG.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingKneeResultPositions[d][index] = sizingLegDeltas.Bones.GetByName(
						pmx.KNEE.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingAnkleResultPositions[d][index] = sizingLegDeltas.Bones.GetByName(
						pmx.ANKLE.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingKneeDResultPositions[d][index] = sizingLegDeltas.Bones.GetByName(
						pmx.KNEE_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingAnkleDResultPositions[d][index] = sizingLegDeltas.Bones.GetByName(
						pmx.ANKLE_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingHeelDResultPositions[d][index] = sizingLegDeltas.Bones.GetByName(
						pmx.HEEL_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
					sizingToeTailDResultPositions[d][index] = sizingLegDeltas.Bones.GetByName(
						pmx.TOE_T_D.StringFromDirection(direction)).FilledGlobalPosition().Copy()
				}
			}

			incrementCompletedCount()

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正07", sizingSet.Index, iterIndex, allCount)
		})

	if mlog.IsDebug() {
		motion := vmd.NewVmdMotion("")
		initialRotations := make([][]*mmath.MQuaternion, 2)
		initialRotations[0] = make([]*mmath.MQuaternion, len(allFrames))
		initialRotations[1] = make([]*mmath.MQuaternion, len(allFrames))

		for i, iFrame := range allFrames {
			frame := float32(iFrame)

			for _, v := range [][]any{
				{legIkPositions, legRotations, "先結%s足IK2"},
				{sizingLegInitialPositions, initialRotations, "先今%s足2"},
				{sizingKneeInitialPositions, initialRotations, "先今%s膝2"},
				{sizingAnkleInitialPositions, initialRotations, "先今%s足首2"},
				{sizingKneeDInitialPositions, initialRotations, "先今%s膝D2"},
				{sizingAnkleDInitialPositions, initialRotations, "先今%s足首D2"},
				{sizingHeelDInitialPositions, initialRotations, "先今%s踵D2"},
				{sizingToeTailDInitialPositions, initialRotations, "先今%s先D2"},
				{sizingAnkleIdealPositions, initialRotations, "先理%s足首2"},
				{sizingToeTargetIdealPositions, initialRotations, "先理%s先D2"},
				{sizingLegResultPositions, initialRotations, "先結%s足2"},
				{sizingKneeResultPositions, initialRotations, "先結%s膝2"},
				{sizingAnkleResultPositions, initialRotations, "先結%s足首2"},
				{sizingKneeDResultPositions, initialRotations, "先結%s膝D2"},
				{sizingAnkleDResultPositions, initialRotations, "先結%s足首D2"},
				{sizingToeTailDResultPositions, initialRotations, "先結%s先D2"},
				{sizingHeelDResultPositions, initialRotations, "先結%s踵D2"},
			} {
				positions := v[0].([][]*mmath.MVec3)
				rotations := v[1].([][]*mmath.MQuaternion)
				for d, direction := range directions {
					boneName := fmt.Sprintf(v[2].(string), direction.String())
					bf := vmd.NewBoneFrame(frame)
					bf.Position = positions[d][i]
					bf.Rotation = rotations[d][i]
					motion.InsertBoneFrame(boneName, bf)
				}
			}
		}

		outputVerboseMotion(verboseMotionKey, sizingSet.OutputMotionPath, motion)
	}

	return legIkPositions, legIkRotations, legRotations, kneeRotations, ankleRotations, nil
}

func (su *SizingLegUsecase) calculateAnkleYDiff(
	originalDelta, sizingDelta *delta.VmdDeltas, originalAnkleBone *pmx.Bone, direction pmx.BoneDirection, ankleScale float64,
) float64 {
	originalToeTailDelta := originalDelta.Bones.GetByName(pmx.TOE_T_D.StringFromDirection(direction))
	originalHeelDelta := originalDelta.Bones.GetByName(pmx.HEEL_D.StringFromDirection(direction))
	sizingToeTailDelta := sizingDelta.Bones.GetByName(pmx.TOE_T_D.StringFromDirection(direction))
	sizingHeelDelta := sizingDelta.Bones.GetByName(pmx.HEEL_D.StringFromDirection(direction))

	originalToeTailDY := originalToeTailDelta.FilledGlobalPosition().Y
	originalHeelDY := originalHeelDelta.FilledGlobalPosition().Y

	// つま先の補正値 ------------------
	// つま先のY座標を元モデルのつま先のY座標*スケールに合わせる
	idealSizingToeTailY := originalToeTailDY * ankleScale

	// 現時点のつま先のY座標(足IKの回転結果を適用させて求め直す)
	actualToeTailY := sizingToeTailDelta.FilledGlobalPosition().Y

	toeDiff := idealSizingToeTailY - actualToeTailY
	lerpToeDiff := mmath.Lerp(toeDiff, 0, originalToeTailDY/originalAnkleBone.Position.Y)

	// かかとの補正値 ------------------
	// かかとのY座標を元モデルのかかとのY座標*スケールに合わせる
	idealSizingHeelY := originalHeelDY * ankleScale

	// 現時点のかかとのY座標
	actualSizingHeelY := sizingHeelDelta.FilledGlobalPosition().Y

	heelDiff := idealSizingHeelY - actualSizingHeelY
	lerpHeelDiff := mmath.Lerp(heelDiff, 0, originalHeelDY/originalAnkleBone.Position.Y)

	// 最終的な差分(つま先のY位置が地面に近いほど、つま先側を優先採用)
	diffY := mmath.Lerp(lerpToeDiff, lerpHeelDiff, originalToeTailDY/originalAnkleBone.Position.Y)

	return diffY
}

// updateLegIkAndFk2 は、計算済みの足IK 補正位置と回転値をモーションデータに反映させます。
func (su *SizingLegUsecase) updateLegIkAndFk2(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	legIkPositions [][]*mmath.MVec3,
	legIkRotations, legRotations, kneeRotations, ankleRotations [][]*mmath.MQuaternion,
	incrementCompletedCount func(),
) {
	for i, iFrame := range allFrames {
		frame := float32(iFrame)
		emptyPositions := make([][]*mmath.MVec3, 2)
		emptyPositions[0] = make([]*mmath.MVec3, len(allFrames))
		emptyPositions[1] = make([]*mmath.MVec3, len(allFrames))

		for _, v := range [][]any{
			{legIkPositions, legIkRotations, pmx.LEG_IK},
			{emptyPositions, legRotations, pmx.LEG},
			{emptyPositions, kneeRotations, pmx.KNEE},
			{emptyPositions, ankleRotations, pmx.ANKLE},
		} {
			positions := v[0].([][]*mmath.MVec3)
			rotations := v[1].([][]*mmath.MQuaternion)
			for d, direction := range directions {
				boneName := v[2].(pmx.StandardBoneName).StringFromDirection(direction)

				bf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
				bf.Position = positions[d][i]
				bf.Rotation = rotations[d][i]
				sizingProcessMotion.InsertBoneFrame(boneName, bf)
			}
		}

		if i > 0 && i%1000 == 0 {
			processLog("足補正08", sizingSet.Index, i, len(allFrames))
		}

		incrementCompletedCount()
	}
}

func (su *SizingLegUsecase) updateOutputMotion(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, sizingProcessMotion *vmd.VmdMotion,
	verboseMotionKey string, incrementCompletedCount func(),
) error {
	// 足補正処理の結果をサイジング先モーションに反映
	sizingModel := sizingSet.SizingConfigModel
	outputMotion := sizingSet.OutputMotion

	targetBoneNames := []string{
		pmx.ROOT.String(), pmx.CENTER.String(), pmx.GROOVE.String(), pmx.LOWER.String(),
		pmx.LEG_IK_PARENT.Left(), pmx.LEG_IK.Left(), pmx.LEG.Left(), pmx.KNEE.Left(), pmx.ANKLE.Left(),
		pmx.LEG_IK_PARENT.Right(), pmx.LEG_IK.Right(), pmx.LEG.Right(), pmx.KNEE.Right(), pmx.ANKLE.Right(),
	}

	activeFrames := getFrames(outputMotion, targetBoneNames)

	// TOE_IK キーフレームのリセット
	outputMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.TOE_IK.Left()))
	outputMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.TOE_IK.Right()))

	// 足系はあるボーンだけ上書きする
	for _, boneName := range targetBoneNames {
		if !outputMotion.BoneFrames.Contains(boneName) {
			continue
		}
		outputMotion.BoneFrames.Get(boneName).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
			processBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
			bf.Position = processBf.FilledPosition().Copy()
			bf.Rotation = processBf.FilledRotation().Copy()
			outputMotion.BoneFrames.Get(boneName).Update(bf)
			return true
		})
	}

	if mlog.IsDebug() {
		outputVerboseMotion(verboseMotionKey, sizingSet.OutputMotionPath, outputMotion)
	}

	// 中間キーフレのズレをチェック
	legThreshold := 0.1
	kneeThreshold := 0.2
	ankleThreshold := 0.1

	err := miter.IterParallelByList(directions, 1, 1,
		func(dIndex int, direction pmx.BoneDirection) error {
			for tIndex, targetFrames := range [][]int{activeFrames, allFrames} {
				processAllDeltas, err := computeVmdDeltas(targetFrames, blockSize,
					sizingModel, sizingProcessMotion, sizingSet, true, all_lower_leg_bone_names, "足補正01", incrementCompletedCount)
				if err != nil {
					return err
				}

				prevLog := 0
				prevFrame := 0
				for fIndex, iFrame := range targetFrames {
					if sizingSet.IsTerminate {
						return merr.TerminateError
					}
					frame := float32(iFrame)

					// 現時点の結果
					resultAllVmdDeltas, err := computeVmdDeltas([]int{iFrame}, 1,
						sizingModel, outputMotion, sizingSet, true, leg_direction_bone_names[dIndex], "", nil)
					if err != nil {
						return err
					}

					// 足の位置をチェック
					resultLegDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.LEG.StringFromDirection(direction))
					processLegDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.LEG.StringFromDirection(direction))

					// ひざの位置をチェック
					resultKneeDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.KNEE.StringFromDirection(direction))
					processKneeDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.KNEE.StringFromDirection(direction))

					// 足首の位置をチェック
					resultAnkleDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))
					processAnkleDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))

					if resultLegDelta.FilledGlobalPosition().Distance(processLegDelta.FilledGlobalPosition()) > legThreshold {
						boneName := pmx.LOWER.String()
						processBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
						resultBf := outputMotion.BoneFrames.Get(boneName).Get(frame)
						resultBf.Rotation = processBf.FilledRotation().Copy()
						outputMotion.InsertBoneFrame(boneName, resultBf)
					}

					// 各関節位置がズレている場合、元の回転を焼き込む
					if resultKneeDelta.FilledGlobalPosition().Distance(processKneeDelta.FilledGlobalPosition()) > kneeThreshold ||
						resultAnkleDelta.FilledGlobalPosition().Distance(processAnkleDelta.FilledGlobalPosition()) > ankleThreshold {

						for _, name := range []pmx.StandardBoneName{pmx.LEG, pmx.KNEE, pmx.ANKLE, pmx.LEG_IK} {
							boneName := name.StringFromDirection(direction)
							processBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
							resultBf := outputMotion.BoneFrames.Get(boneName).Get(frame)
							resultBf.Position = processBf.FilledPosition().Copy()
							resultBf.Rotation = processBf.FilledRotation().Copy()
							outputMotion.InsertBoneFrame(boneName, resultBf)
						}
					}

					if fIndex > 0 && int(iFrame/1000) > prevLog {
						mlog.I(mi18n.T("足補正11", map[string]interface{}{
							"No":          sizingSet.Index + 1,
							"IterIndex":   fmt.Sprintf("%04d", iFrame),
							"AllCount":    fmt.Sprintf("%04d", allFrames[len(allFrames)-1]),
							"Direction":   direction.String(),
							"FramesIndex": tIndex + 1}))
						prevLog = int(iFrame / 1000)
					}

					for f := prevFrame + 1; f <= iFrame; f++ {
						incrementCompletedCount()
					}

					prevFrame = iFrame
				}
			}

			return nil
		}, nil)
	if err != nil {
		return err
	}

	return nil
}

func (su *SizingLegUsecase) updateLegIkOffset(sizingSet *domain.SizingSet, allFrames []int) {
	// 元モーションのIKが動いていない区間を取得
	fixIkFlags := make([][]bool, len(allFrames))
	for d, direction := range directions {
		boneName := pmx.LEG_IK.StringFromDirection(direction)
		fixIkFlags[d] = make([]bool, len(allFrames))

		for i, iFrame := range allFrames {
			if i == 0 {
				continue
			}
			frame := float32(iFrame)
			bf := sizingSet.OriginalMotion.BoneFrames.Get(boneName).Get(frame)
			prevBf := sizingSet.OriginalMotion.BoneFrames.Get(boneName).Get(frame - 1)
			fixIkFlags[d][i] = bf.FilledPosition().NearEquals(prevBf.FilledPosition(), 1e-2)
		}
	}

	// IKのオフセットを元に戻す
	for d, direction := range directions {
		boneName := pmx.LEG_IK.StringFromDirection(direction)
		sizingSet.OutputMotion.BoneFrames.Get(boneName).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
			bf.Position.Z += 0.1
			if fixIkFlags[d][int(frame)] {
				// 固定されている場合、前のキーフレを引き継ぐ
				prevFrame := sizingSet.OutputMotion.BoneFrames.Get(boneName).PrevFrame(frame)
				prevBf := sizingSet.OutputMotion.BoneFrames.Get(boneName).Get(prevFrame)
				bf.Position = prevBf.FilledPosition().Copy()
			}
			sizingSet.OutputMotion.BoneFrames.Get(boneName).Update(bf)

			return true
		})
	}
}

func (su *SizingLegUsecase) createLowerBoneNames(sizingSet *domain.SizingSet) []string {
	leftToeIkBone := sizingSet.SizingLeftToeIkBone()
	rightToeIkBone := sizingSet.SizingRightToeIkBone()
	leftToeIkTargetBone, _ := sizingSet.SizingConfigModel.Bones.Get(leftToeIkBone.Ik.BoneIndex)
	rightToeIkTargetBone, _ := sizingSet.SizingConfigModel.Bones.Get(rightToeIkBone.Ik.BoneIndex)

	return append(all_lower_leg_bone_names, leftToeIkTargetBone.Name(), rightToeIkTargetBone.Name())
}

func (su *SizingLegUsecase) checkBones(sizingSet *domain.SizingSet) (err error) {

	for _, v := range [][]interface{}{
		{sizingSet.OriginalCenterBone, pmx.CENTER.String(), true},
		{sizingSet.OriginalLowerBone, pmx.LOWER.String(), true},
		{sizingSet.OriginalBodyAxisBone, pmx.BODY_AXIS.String(), false},
		{sizingSet.OriginalLegCenterBone, pmx.LEG_CENTER.String(), false},
		{sizingSet.OriginalTrunkRootBone, pmx.TRUNK_ROOT.String(), false},
		{sizingSet.OriginalLeftArmBone, pmx.ARM.Left(), true},
		{sizingSet.OriginalRightArmBone, pmx.ARM.Right(), true},
		{sizingSet.OriginalLeftLegIkBone, pmx.LEG_IK.Left(), true},
		{sizingSet.OriginalLeftLegRootBone, pmx.LEG_ROOT.Left(), false},
		{sizingSet.OriginalLeftLegBone, pmx.LEG.Left(), true},
		{sizingSet.OriginalLeftKneeBone, pmx.KNEE.Left(), true},
		{sizingSet.OriginalLeftAnkleBone, pmx.ANKLE.Left(), true},
		{sizingSet.OriginalLeftAnkleDBone, pmx.ANKLE_D.Left(), true},
		{sizingSet.OriginalLeftAnkleDGroundBone, pmx.ANKLE_D_GROUND.Left(), false},
		{sizingSet.OriginalLeftToeIkBone, pmx.TOE_IK.Left(), true},
		{sizingSet.OriginalLeftToeTailDBone, pmx.TOE_T_D.Left(), false},
		{sizingSet.OriginalLeftHeelDBone, pmx.HEEL_D.Left(), false},
		{sizingSet.OriginalLeftToePDBone, pmx.TOE_P_D.Left(), false},
		{sizingSet.OriginalRightLegIkBone, pmx.LEG_IK.Right(), true},
		{sizingSet.OriginalRightLegRootBone, pmx.LEG_ROOT.Right(), false},
		{sizingSet.OriginalRightLegBone, pmx.LEG.Right(), true},
		{sizingSet.OriginalRightKneeBone, pmx.KNEE.Right(), true},
		{sizingSet.OriginalRightAnkleBone, pmx.ANKLE.Right(), true},
		{sizingSet.OriginalRightAnkleDBone, pmx.ANKLE_D.Right(), true},
		{sizingSet.OriginalRightAnkleDGroundBone, pmx.ANKLE_D_GROUND.Right(), false},
		{sizingSet.OriginalRightToeIkBone, pmx.TOE_IK.Right(), true},
		{sizingSet.OriginalRightToeTailDBone, pmx.TOE_T_D.Right(), false},
		{sizingSet.OriginalRightHeelDBone, pmx.HEEL_D.Right(), false},
		{sizingSet.OriginalRightToePDBone, pmx.TOE_P_D.Right(), false},
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

	for _, v := range [][]interface{}{
		{sizingSet.SizingCenterBone, pmx.CENTER.String(), true},
		{sizingSet.SizingLowerBone, pmx.LOWER.String(), true},
		{sizingSet.SizingBodyAxisBone, pmx.BODY_AXIS.String(), false},
		{sizingSet.SizingLegCenterBone, pmx.LEG_CENTER.String(), false},
		{sizingSet.SizingTrunkRootBone, pmx.TRUNK_ROOT.String(), false},
		{sizingSet.SizingLeftArmBone, pmx.ARM.Left(), true},
		{sizingSet.SizingRightArmBone, pmx.ARM.Right(), true},
		{sizingSet.SizingLeftLegIkBone, pmx.LEG_IK.Left(), true},
		{sizingSet.SizingLeftLegBone, pmx.LEG.Left(), true},
		{sizingSet.SizingLeftKneeBone, pmx.KNEE.Left(), true},
		{sizingSet.SizingLeftAnkleBone, pmx.ANKLE.Left(), true},
		{sizingSet.SizingLeftAnkleDBone, pmx.ANKLE_D.Left(), true},
		{sizingSet.SizingLeftAnkleDGroundBone, pmx.ANKLE_D_GROUND.Left(), false},
		{sizingSet.SizingLeftToeIkBone, pmx.TOE_IK.Left(), true},
		{sizingSet.SizingLeftToeTailDBone, pmx.TOE_T_D.Left(), false},
		{sizingSet.SizingLeftHeelDBone, pmx.HEEL_D.Left(), false},
		{sizingSet.SizingLeftToePDBone, pmx.TOE_P_D.Left(), false},
		{sizingSet.SizingRightLegIkBone, pmx.LEG_IK.Right(), true},
		{sizingSet.SizingRightLegBone, pmx.LEG.Right(), true},
		{sizingSet.SizingRightKneeBone, pmx.KNEE.Right(), true},
		{sizingSet.SizingRightAnkleBone, pmx.ANKLE.Right(), true},
		{sizingSet.SizingRightAnkleDBone, pmx.ANKLE_D.Right(), true},
		{sizingSet.SizingRightAnkleDGroundBone, pmx.ANKLE_D_GROUND.Right(), false},
		{sizingSet.SizingRightToeIkBone, pmx.TOE_IK.Right(), true},
		{sizingSet.SizingRightToeTailDBone, pmx.TOE_T_D.Right(), false},
		{sizingSet.SizingRightHeelDBone, pmx.HEEL_D.Right(), false},
		{sizingSet.SizingRightToePDBone, pmx.TOE_P_D.Right(), false},
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
