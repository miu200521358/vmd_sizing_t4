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

	allFrames := mmath.IntRanges(int(originalMotion.MaxFrame()))
	blockSize, _ := miter.GetBlockSize(len(allFrames) * sizingSetCount)

	// [焼き込み] -----------------------

	// 元モデルのデフォーム結果を並列処理で取得
	originalAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.OriginalConfigModel,
		originalMotion, sizingSet, true, all_lower_leg_bone_names, "足補正01", incrementCompletedCount)
	if err != nil {
		return false, err
	}

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

	// サイジング先モデルに対して FK 焼き込み処理
	if err := su.updateLegFK(sizingSet, sizingProcessMotion, originalAllDeltas, incrementCompletedCount); err != nil {
		return false, err
	}

	if mlog.IsDebug() {
		su.insertIKFrames(sizingSet, sizingProcessMotion, false)
		outputVerboseMotion("足01", sizingSet.OutputMotionPath, sizingProcessMotion)
	}

	lowerBoneNames := su.createLowerBoneNames(sizingSet)

	// [下半身] -----------------------

	// 先モデルの足中心までのデフォーム結果を並列処理で取得
	sizingAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel,
		sizingProcessMotion, sizingSet, false, trunk_lower_bone_names, "足補正01", incrementCompletedCount)
	if err != nil {
		return false, err
	}

	// 下半身補正を実施
	lowerRotations, err := su.calculateAdjustedLower(sizingSet, allFrames, blockSize,
		originalAllDeltas, sizingAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas,
		sizingProcessMotion, incrementCompletedCount, "足02")
	if err != nil {
		return false, err
	}

	// 下半身回転をサイジング先モーションに反映
	su.updateLower(sizingSet, allFrames, sizingProcessMotion, lowerRotations, incrementCompletedCount)

	if mlog.IsDebug() {
		outputVerboseMotion("足03", sizingSet.OutputMotionPath, sizingProcessMotion)
	}

	// [足IK] -----------------------

	// 先モデルの足のIK OFF状態でのデフォーム結果を並列処理で取得
	sizingAllDeltas, err = computeVmdDeltasWithDeltas(allFrames, blockSize, sizingSet.SizingConfigModel,
		sizingProcessMotion, sizingAllDeltas, sizingSet, false, lowerBoneNames, "足補正01", incrementCompletedCount)
	if err != nil {
		return false, err
	}

	// 足IK 補正処理
	legIkParentPositions, legIkPositions,
		legIkParentRotations, legIkRotations, legRotations, ankleRotations, err :=
		su.calculateAdjustedLegIK(sizingSet, allFrames, blockSize, moveScale,
			originalAllDeltas, sizingAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas,
			sizingProcessMotion, incrementCompletedCount, "足04")
	if err != nil {
		return false, err
	}

	// 足IKの位置と足の角度 をサイジング先モーションに反映
	su.updateLegIkAndFk(sizingSet, allFrames, sizingProcessMotion,
		legIkParentPositions, legIkPositions, legIkParentRotations,
		legIkRotations, legRotations, ankleRotations, incrementCompletedCount)

	// TOE_IK キーフレームのリセット(つま先IKの計算が終わったのでここでリセット)
	sizingProcessMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.TOE_IK.Left()))
	sizingProcessMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.TOE_IK.Right()))

	if mlog.IsDebug() {
		outputVerboseMotion("足05", sizingSet.OutputMotionPath, sizingProcessMotion)
	}

	// [センター] -----------------------

	// 先モデルの足FK補正デフォーム結果を並列処理で取得
	sizingAllDeltas, err = computeVmdDeltasWithDeltas(allFrames, blockSize, sizingSet.SizingConfigModel,
		sizingProcessMotion, sizingAllDeltas, sizingSet, false, lowerBoneNames, "足補正01", incrementCompletedCount)
	if err != nil {
		return false, err
	}

	// センター・グルーブ補正を実施
	rootPositions, centerPositions, groovePositions, err :=
		su.calculateAdjustedCenter(sizingSet, allFrames, blockSize, moveScale,
			originalAllDeltas, sizingAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas,
			sizingProcessMotion, incrementCompletedCount, "足06")
	if err != nil {
		return false, err
	}

	// センター・グルーブ位置をサイジング先モーションに反映
	su.updateCenter(sizingSet, allFrames, sizingProcessMotion,
		rootPositions, centerPositions, groovePositions,
		incrementCompletedCount)

	if mlog.IsDebug() {
		outputVerboseMotion("足07", sizingSet.OutputMotionPath, sizingProcessMotion)
	}

	// [足IK2回目] -----------------------

	// 先モデルの足FK補正デフォーム結果を並列処理で取得
	sizingLegIkOffAllDeltas, err := computeVmdDeltasWithDeltas(allFrames, blockSize, sizingSet.SizingConfigModel,
		sizingProcessMotion, sizingAllDeltas, sizingSet, false, lowerBoneNames, "足補正01", incrementCompletedCount)
	if err != nil {
		return false, err
	}

	// 先モデルの足FK補正デフォーム結果を並列処理で取得
	sizingLegIkOnAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel,
		sizingProcessMotion, sizingSet, true, lowerBoneNames, "足補正01", incrementCompletedCount)
	if err != nil {
		return false, err
	}

	// 足IK 補正処理
	legIkPositions, legIkRotations, legRotations, kneeRotations, ankleRotations, err :=
		su.calculateAdjustedLegIK2(sizingSet, allFrames, blockSize, moveScale,
			originalAllDeltas, sizingLegIkOffAllDeltas, sizingLegIkOnAllDeltas,
			originalMorphAllDeltas, sizingMorphAllDeltas,
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

	// [結果出力] -----------------------

	// あにまさミクを基準に、近似値のスケールを求める
	legScale := sizingSet.SizingLegCenterBone().Position.Y / 10

	// 足補正処理の結果をサイジング先モーションに反映
	if err = su.updateOutputMotion(
		sizingSet, allFrames, blockSize, sizingProcessMotion, legScale, "足10",
		incrementCompletedCount,
	); err != nil {
		return false, err
	}

	if mlog.IsDebug() {
		outputVerboseMotion("足11", sizingSet.OutputMotionPath, sizingSet.OutputMotion)
	}

	// 足IKのオフセットを元に戻す
	su.updateLegIkOffset(sizingSet, allFrames, legScale)

	if mlog.IsDebug() {
		outputVerboseMotion("足12", sizingSet.OutputMotionPath, sizingSet.OutputMotion)
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
			return merr.NewTerminateError("manual terminate")
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
	kf.IkList = append(kf.IkList, su.newIkEnableFrameWithBone(
		sizingSet.SizingLegIkBone(pmx.BONE_DIRECTION_LEFT).Name(), enabled))
	kf.IkList = append(kf.IkList, su.newIkEnableFrameWithBone(
		sizingSet.SizingToeIkBone(pmx.BONE_DIRECTION_LEFT).Name(), enabled))
	// 右足
	kf.IkList = append(kf.IkList, su.newIkEnableFrameWithBone(
		sizingSet.SizingLegIkBone(pmx.BONE_DIRECTION_RIGHT).Name(), enabled))
	kf.IkList = append(kf.IkList, su.newIkEnableFrameWithBone(
		sizingSet.SizingToeIkBone(pmx.BONE_DIRECTION_RIGHT).Name(), enabled))
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
	ikBone.Ik.LoopCount = 10
	ikBone.Ik.UnitRotation = &mmath.MVec3{X: 1, Y: 0.0, Z: 0.0}
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
	sizingProcessMotion *vmd.VmdMotion, incrementCompletedCount func(), debugMotionKey string,
) (sizingLowerResultRotations []*mmath.MQuaternion, err error) {
	sizingLowerResultRotations = make([]*mmath.MQuaternion, len(allFrames))

	lowerIkBone := su.createLowerIkBone(sizingSet, pmx.BONE_DIRECTION_TRUNK)

	debugBoneNames := []pmx.StandardBoneName{
		pmx.LOWER, pmx.LEG_CENTER, pmx.LEG,
	}
	debugPositions, debugRotations := newDebugData(allFrames, debugBoneNames)

	err = miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, iFrame int) error {
			if sizingSet.IsTerminate {
				return merr.NewTerminateError("manual terminate")
			}
			frame := float32(iFrame)

			// 下半身から足中心の傾き
			originalMorphLowerDelta := originalMorphAllDeltas[index].Bones.GetByName(pmx.LOWER.String())
			originalMorphLegCenterDelta := originalMorphAllDeltas[index].Bones.GetByName(pmx.LEG_CENTER.String())
			originalMorphLeftLegDelta := originalMorphAllDeltas[index].Bones.GetByName(pmx.LEG.Left())
			originalMorphRightLegDelta := originalMorphAllDeltas[index].Bones.GetByName(pmx.LEG.Right())
			sizingMorphLowerDelta := sizingMorphAllDeltas[index].Bones.GetByName(pmx.LOWER.String())
			sizingMorphLegCenterDelta := sizingMorphAllDeltas[index].Bones.GetByName(pmx.LEG_CENTER.String())
			sizingMorphLeftLegDelta := sizingMorphAllDeltas[index].Bones.GetByName(pmx.LEG.Left())
			sizingMorphRightLegDelta := sizingMorphAllDeltas[index].Bones.GetByName(pmx.LEG.Right())

			// 下半身と足中心の相対位置
			originalMorphLegCenterRelativePosition := originalMorphLegCenterDelta.FilledGlobalPosition().Subed(
				originalMorphLowerDelta.FilledGlobalPosition())
			// sizingMorphLegCenterRelativePosition := sizingMorphLegCenterDelta.FilledGlobalPosition().Subed(
			// sizingMorphLowerDelta.FilledGlobalPosition())

			// 真下から足中心までの傾き
			originalLegSlope := mmath.NewMQuaternionRotate(mmath.MVec3UnitYNeg, originalMorphLegCenterRelativePosition.Normalized())
			// sizingLegSlope := mmath.NewMQuaternionRotate(mmath.MVec3UnitYNeg, sizingMorphLegCenterRelativePosition.Normalized())

			// 下半身の長さ
			originalLowerHeight := originalMorphLowerDelta.FilledGlobalPosition().Distance(originalMorphLegCenterDelta.FilledGlobalPosition())
			sizingLowerHeight := sizingMorphLowerDelta.FilledGlobalPosition().Distance(sizingMorphLegCenterDelta.FilledGlobalPosition())

			// 足幅
			originalLegWidth := originalMorphLeftLegDelta.FilledGlobalPosition().Distance(originalMorphRightLegDelta.FilledGlobalPosition())
			sizingLegWidth := sizingMorphLeftLegDelta.FilledGlobalPosition().Distance(sizingMorphRightLegDelta.FilledGlobalPosition())

			legCenterFromLowerScale := &mmath.MVec3{
				X: sizingLegWidth / originalLegWidth,
				Y: sizingLowerHeight / originalLowerHeight,
				Z: 1.0,
			}

			// 足中心の長さ差
			originalLegCenterHeight := originalMorphLegCenterDelta.FilledGlobalPosition().Y
			sizingLegCenterHeight := sizingMorphLegCenterDelta.FilledGlobalPosition().Y
			legCenterRatio := sizingLegCenterHeight / originalLegCenterHeight

			// 実際に傾ける大きさは、比率が小さいほど小さくする
			slope := mmath.MQuaternionIdent.Slerp(originalLegSlope, legCenterRatio)

			// -------------------------

			originalLowerRootDelta := originalAllDeltas[index].Bones.GetByName(pmx.LOWER_ROOT.String())
			originalLowerDelta := originalAllDeltas[index].Bones.GetByName(pmx.LOWER.String())
			originalLegCenterDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG_CENTER.String())

			// 下半身の軸回転を取得
			lowerTwistQuat, _ := originalLowerDelta.FilledFrameRotation().SeparateTwistByAxis(mmath.MVec3UnitYNeg)
			lowerTwistMat := lowerTwistQuat.ToMat4()

			// 下半身根元に下半身の軸回転を加えたところから見た足ボーンのローカル位置
			originalLegCenterLocalPosition := originalLowerRootDelta.FilledGlobalMatrix().Copy().Muled(
				lowerTwistMat).Inverted().MulVec3(originalLegCenterDelta.FilledGlobalPosition())

			// 真っ直ぐにしたときのローカル位置
			originalLegCenterVerticalLocalPosition := originalLegSlope.Inverted().MulVec3(originalLegCenterLocalPosition).Truncate(1e-3)

			// スケール差を考慮した先の足ボーンのローカル位置
			sizingLegCenterVerticalLocalPosition := originalLegCenterVerticalLocalPosition.Muled(legCenterFromLowerScale)

			sizingLegCenterLocalPosition := slope.MulVec3(sizingLegCenterVerticalLocalPosition)

			sizingLowerRootDelta := sizingAllDeltas[index].Bones.GetByName(pmx.LOWER_ROOT.String())
			sizingLegCenterIdealGlobalPosition := sizingLowerRootDelta.FilledGlobalMatrix().Muled(
				lowerTwistMat).MulVec3(sizingLegCenterLocalPosition)

			if mlog.IsDebug() {
				recordDebugData(index, debugBoneNames, originalAllDeltas[index],
					debugTargetOriginal, debugTypeInitial, debugPositions, debugRotations)
				recordDebugData(index, debugBoneNames, sizingAllDeltas[index],
					debugTargetSizing, debugTypeInitial, debugPositions, debugRotations)

				debugPositions[debugTargetSizing][debugTypeIdeal][pmx.LEG_CENTER.String()][index] = sizingLegCenterIdealGlobalPosition.Copy()
			}

			// IK解決
			sizingLowerDeltas, _ := deform.DeformIks(sizingSet.SizingConfigModel, sizingProcessMotion,
				sizingAllDeltas[index], frame,
				[]*pmx.Bone{lowerIkBone},
				[]*pmx.Bone{sizingSet.SizingLegCenterBone()},
				[]*mmath.MVec3{sizingLegCenterIdealGlobalPosition},
				trunk_lower_bone_names, 2, false, false)

			sizingLowerResultDelta := sizingLowerDeltas.Bones.GetByName(pmx.LOWER.String())
			sizingLowerResultRotations[index] = sizingLowerResultDelta.FilledFrameRotation().Copy()

			if mlog.IsDebug() {
				recordDebugData(index, debugBoneNames, sizingLowerDeltas,
					debugTargetSizing, debugTypeResult, debugPositions, debugRotations)
			}

			{
				// デフォーム情報を更新するため、クリア
				sizingLowerDelta := sizingAllDeltas[index].Bones.GetByName(pmx.LOWER.String())
				sizingLowerDelta.UnitMatrix = nil
				sizingAllDeltas[index].Bones.Update(sizingLowerDelta)
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
		outputDebugData(allFrames, debugBoneNames, debugMotionKey, sizingSet.OutputMotionPath, sizingSet.SizingConfigModel, debugPositions, debugRotations)
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

func (su *SizingLegUsecase) createKneeIkBone(sizingSet *domain.SizingSet, direction pmx.BoneDirection) (
	ikBone *pmx.Bone,
) {
	legBone, _ := sizingSet.SizingConfigModel.Bones.GetLeg(direction)
	tailBone, _ := sizingSet.SizingConfigModel.Bones.GetKneeD(direction)
	ikBone = pmx.NewBoneByName(fmt.Sprintf("%s%sIk", pmx.MLIB_PREFIX, tailBone.Name()))

	ikBone.Position = tailBone.Position.Copy()
	ikBone.Ik = pmx.NewIk()
	ikBone.Ik.BoneIndex = tailBone.Index()
	ikBone.Ik.LoopCount = 100
	ikBone.Ik.UnitRotation = &mmath.MVec3{X: 0.1, Y: 0.0, Z: 0.0}
	ikBone.Ik.Links = make([]*pmx.IkLink, 0)

	for _, boneName := range []string{
		pmx.LEG_D.StringFromDirection(direction),
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
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, moveScale *mmath.MVec3,
	originalAllDeltas, sizingAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas []*delta.VmdDeltas,
	sizingProcessMotion *vmd.VmdMotion, incrementCompletedCount func(), verboseMotionKey string,
) (legIkParentPositions, legIkPositions [][]*mmath.MVec3,
	legIkParentRotations, legIkRotations, legRotations, ankleRotations [][]*mmath.MQuaternion, err error) {
	legIkParentPositions = make([][]*mmath.MVec3, 2)
	legIkPositions = make([][]*mmath.MVec3, 2)
	legIkParentRotations = make([][]*mmath.MQuaternion, 2)
	legIkRotations = make([][]*mmath.MQuaternion, 2)
	legRotations = make([][]*mmath.MQuaternion, 2)
	ankleRotations = make([][]*mmath.MQuaternion, 2)

	for i := range directions {
		legIkParentPositions[i] = make([]*mmath.MVec3, len(allFrames))
		legIkPositions[i] = make([]*mmath.MVec3, len(allFrames))
		legIkParentRotations[i] = make([]*mmath.MQuaternion, len(allFrames))
		legIkRotations[i] = make([]*mmath.MQuaternion, len(allFrames))
		legRotations[i] = make([]*mmath.MQuaternion, len(allFrames))
		ankleRotations[i] = make([]*mmath.MQuaternion, len(allFrames))
	}

	kneeIkBones := make([]*pmx.Bone, 2)
	kneeIkBones[0] = su.createKneeIkBone(sizingSet, pmx.BONE_DIRECTION_LEFT)
	kneeIkBones[1] = su.createKneeIkBone(sizingSet, pmx.BONE_DIRECTION_RIGHT)

	toeIkTargetBones := make([]*pmx.Bone, 2)
	toeIkTargetBones[0], _ = sizingSet.SizingConfigModel.Bones.Get(
		sizingSet.SizingToeIkBone(pmx.BONE_DIRECTION_LEFT).Ik.BoneIndex)
	toeIkTargetBones[1], _ = sizingSet.SizingConfigModel.Bones.Get(
		sizingSet.SizingToeIkBone(pmx.BONE_DIRECTION_RIGHT).Ik.BoneIndex)

	debugBoneNames := []pmx.StandardBoneName{
		pmx.LEG_CENTER, pmx.LEG, pmx.KNEE, pmx.ANKLE,
	}
	debugPositions, debugRotations := newDebugData(allFrames, debugBoneNames)

	err = miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.NewTerminateError("manual terminate")
			}

			for d, direction := range directions {
				// 腰骨から膝までの長さ差
				originalMorphHipDelta := originalMorphAllDeltas[index].Bones.GetByName(
					pmx.HIP.StringFromDirection(direction))
				// originalMorphLegDelta := originalMorphAllDeltas[index].Bones.GetByName(
				// 	pmx.LEG.StringFromDirection(direction))
				originalMorphKneeDelta := originalMorphAllDeltas[index].Bones.GetByName(
					pmx.KNEE.StringFromDirection(direction))
				// originalMorphAnkleDelta := originalMorphAllDeltas[index].Bones.GetByName(
				// 	pmx.ANKLE.StringFromDirection(direction))
				sizingMorphHipDelta := sizingMorphAllDeltas[index].Bones.GetByName(
					pmx.HIP.StringFromDirection(direction))
				// sizingMorphLegDelta := sizingMorphAllDeltas[index].Bones.GetByName(
				// 	pmx.LEG.StringFromDirection(direction))
				sizingMorphKneeDelta := sizingMorphAllDeltas[index].Bones.GetByName(
					pmx.KNEE.StringFromDirection(direction))
				// sizingMorphAnkleDelta := sizingMorphAllDeltas[index].Bones.GetByName(
				// 	pmx.ANKLE.StringFromDirection(direction))

				originalKneeLength := originalMorphHipDelta.FilledGlobalPosition().Distance(
					originalMorphKneeDelta.FilledGlobalPosition())
				sizingKneeLength := sizingMorphHipDelta.FilledGlobalPosition().Distance(
					sizingMorphKneeDelta.FilledGlobalPosition())

				// // 足の長さの比率
				// sizingLegScale := sizingLegLength / originalLegLength
				// // 足の長さの比率と同じ場合の腰骨の長さ
				// sizingIdealHipLength := originalHipLength * sizingLegScale
				// // 想定腰骨の長さと、実際の腰骨の長さの比率
				// sizingHipScale := sizingHipLength / sizingIdealHipLength

				sizingHip2KneeScale := sizingKneeLength / originalKneeLength

				// -------------------------------

				originalHipDelta := originalAllDeltas[index].Bones.GetByName(
					pmx.HIP.StringFromDirection(direction))
				originalKneeDDelta := originalAllDeltas[index].Bones.GetByName(
					pmx.KNEE_D.StringFromDirection(direction))

				sizingHipDelta := sizingAllDeltas[index].Bones.GetByName(
					pmx.HIP.StringFromDirection(direction))

				// 腰骨ボーンから見たひざまでの相対位置
				originalKneeLocalPosition := originalKneeDDelta.FilledGlobalPosition().Subed(
					originalHipDelta.FilledGlobalPosition())

				// スケール差を考慮した先のひざボーンのローカル位置
				sizingKneeLocalPosition := originalKneeLocalPosition.MuledScalar(sizingHip2KneeScale)

				sizingKneeIdealGlobalPosition := sizingHipDelta.FilledGlobalPosition().Added(sizingKneeLocalPosition)

				// // つま先ターゲットの理想位置を求める
				// sizingToeTargetInitialGlobalPosition := sizingAllDeltas[index].Bones.Get(toeIkTargetBones[d].Index()).FilledGlobalPosition()
				// sizingAnkleInitialGlobalPosition := sizingAllDeltas[index].Bones.GetByName(pmx.ANKLE.StringFromDirection(direction)).FilledGlobalPosition()
				// sizingToeTargetRelativePosition := sizingToeTargetInitialGlobalPosition.Subed(sizingAnkleInitialGlobalPosition)
				// sizingToeTargetIdealGlobalPosition := sizingKneeIdealGlobalPosition.Added(sizingToeTargetRelativePosition)

				if mlog.IsDebug() {
					recordDebugData(index, debugBoneNames, originalAllDeltas[index],
						debugTargetOriginal, debugTypeInitial, debugPositions, debugRotations)
					recordDebugData(index, debugBoneNames, sizingAllDeltas[index],
						debugTargetSizing, debugTypeInitial, debugPositions, debugRotations)

					debugPositions[debugTargetSizing][debugTypeIdeal][pmx.KNEE.StringFromDirection(direction)][index] = sizingKneeIdealGlobalPosition.Copy()
				}

				// IK解決
				sizingLegDeltas, deformBoneIndexes := deform.DeformIks(sizingSet.SizingConfigModel, sizingProcessMotion,
					sizingAllDeltas[index], float32(data),
					[]*pmx.Bone{kneeIkBones[d]},
					[]*pmx.Bone{sizingSet.SizingKneeBone(direction)},
					[]*mmath.MVec3{sizingKneeIdealGlobalPosition},
					leg_direction_bone_names[d], 1, false, false)

				// 足IK親が有効な場合、足IK親のローカル位置を比率ベースで求め直す
				sizingLegIkParentDelta := sizingLegDeltas.Bones.GetByName(pmx.LEG_IK_PARENT.StringFromDirection(direction))

				if sizingLegIkParentDelta != nil && (!sizingLegIkParentDelta.FilledFramePosition().IsZero() ||
					!sizingLegIkParentDelta.FilledFrameRotation().IsIdent()) {
					legIkParentPositions[d][index] = sizingLegIkParentDelta.FilledFramePosition().Muled(moveScale)
					legIkParentRotations[d][index] = sizingLegIkParentDelta.FilledFrameRotation().Copy()

					// デフォーム情報を更新
					sizingLegIkParentDelta.FramePosition = legIkParentPositions[d][index].Copy()
					sizingLegIkParentDelta.UnitMatrix = nil
					sizingLegDeltas.Bones.Update(sizingLegIkParentDelta)
					deform.UpdateGlobalMatrix(sizingLegDeltas.Bones, deformBoneIndexes)
				}

				// 足IKの親からみた足IKデフォルト位置からみた現在の足首のローカル位置
				{
					legIkParentBoneIndex := sizingSet.SizingLegIkBone(direction).ParentBone.Index()
					sizingLegIkMorphDelta := sizingMorphAllDeltas[index].Bones.GetByName(pmx.LEG_IK.StringFromDirection(direction))
					sizingLegIkParentMorphDelta := sizingMorphAllDeltas[index].Bones.Get(legIkParentBoneIndex)
					sizingLegIkMorphLocalPosition :=
						sizingLegIkParentMorphDelta.FilledGlobalMatrix().Inverted().MulVec3(
							sizingLegIkMorphDelta.FilledGlobalPosition())

					sizingAnkleDelta := sizingLegDeltas.Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))
					sizingLegIkParentDelta := sizingLegDeltas.Bones.Get(legIkParentBoneIndex)
					legIkPositions[d][index] = sizingLegIkParentDelta.FilledGlobalMatrix().Inverted().MulVec3(sizingAnkleDelta.FilledGlobalPosition()).Subed(sizingLegIkMorphLocalPosition)
					legIkRotations[d][index] = sizingLegDeltas.Bones.GetByName(pmx.LEG_IK.StringFromDirection(direction)).FilledFrameRotation()
				}

				legDelta := sizingLegDeltas.Bones.GetByName(pmx.LEG.StringFromDirection(direction))
				legRotations[d][index] = legDelta.FilledFrameRotation().Copy()

				ankleDelta := sizingLegDeltas.Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))
				ankleRotations[d][index] = ankleDelta.FilledFrameRotation().Copy()

				if mlog.IsDebug() {
					recordDebugData(index, debugBoneNames, sizingLegDeltas,
						debugTargetSizing, debugTypeResult, debugPositions, debugRotations)
				}

				{
					// デフォーム情報を更新するため、クリア
					for _, boneName := range []pmx.StandardBoneName{
						pmx.LEG_IK_PARENT, pmx.LEG_IK, pmx.LEG, pmx.ANKLE,
					} {
						sizingDelta := sizingAllDeltas[index].Bones.GetByName(boneName.StringFromDirection(direction))
						if sizingDelta == nil {
							continue
						}
						sizingDelta.UnitMatrix = nil
						sizingAllDeltas[index].Bones.Update(sizingDelta)
					}
				}
			}

			incrementCompletedCount()

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正07", sizingSet.Index, iterIndex, allCount)
		})
	if err != nil {
		return nil, nil, nil, nil, nil, nil, err
	}

	if mlog.IsDebug() {
		outputDebugData(allFrames, debugBoneNames, verboseMotionKey, sizingSet.OutputMotionPath, sizingSet.SizingConfigModel, debugPositions, debugRotations)
	}

	return legIkParentPositions, legIkPositions, legIkParentRotations, legIkRotations, legRotations, ankleRotations, nil
}

// updateLegIkAndFk は、計算済みの足IK 補正位置と回転値をモーションデータに反映させます。
func (su *SizingLegUsecase) updateLegIkAndFk(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	legIkParentPositions, legIkPositions [][]*mmath.MVec3,
	legIkParentRotations, legIkRotations, legRotations, ankleRotations [][]*mmath.MQuaternion,
	incrementCompletedCount func(),
) {
	emptyPositions := make([][]*mmath.MVec3, 2)
	emptyPositions[0] = make([]*mmath.MVec3, len(allFrames))
	emptyPositions[1] = make([]*mmath.MVec3, len(allFrames))

	for i, iFrame := range allFrames {
		frame := float32(iFrame)

		for _, v := range [][]any{
			{legIkParentPositions, legIkParentRotations, pmx.LEG_IK_PARENT},
			{legIkPositions, legIkRotations, pmx.LEG_IK},
			{emptyPositions, legRotations, pmx.LEG},
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

// calculateAdjustedCenter は、センターおよびグルーブの位置補正を並列処理で計算します。
func (su *SizingLegUsecase) calculateAdjustedCenter(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, moveScale *mmath.MVec3,
	originalAllDeltas, sizingAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas []*delta.VmdDeltas,
	sizingProcessMotion *vmd.VmdMotion, incrementCompletedCount func(), debugMotionKey string,
) (
	rootPositions, centerPositions, groovePositions []*mmath.MVec3,
	err error,
) {
	rootPositions = make([]*mmath.MVec3, len(allFrames))
	centerPositions = make([]*mmath.MVec3, len(allFrames))
	groovePositions = make([]*mmath.MVec3, len(allFrames))

	isActiveGroove := false
	sizingProcessMotion.BoneFrames.Get(pmx.GROOVE.String()).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
		if !mmath.NearEquals(bf.FilledPosition().Y, 0.0, 1e-3) {
			isActiveGroove = true
			return false
		}
		return true
	})

	debugBoneNames := []pmx.StandardBoneName{
		pmx.BODY_AXIS,
	}
	debugPositions, debugRotations := newDebugData(allFrames, debugBoneNames)

	err = miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.NewTerminateError("manual terminate")
			}

			// 体軸までのYの長さ
			originalMorphBodyAxisDelta := originalMorphAllDeltas[index].Bones.GetByName(pmx.BODY_AXIS.String())
			sizingMorphBodyAxisDelta := sizingMorphAllDeltas[index].Bones.GetByName(pmx.BODY_AXIS.String())

			// originalLeftAnkleDelta := originalMorphAllDeltas[index].Bones.GetByName(pmx.ANKLE.Left())
			// originalRightAnkleDelta := originalMorphAllDeltas[index].Bones.GetByName(pmx.ANKLE.Right())
			// originalAnkleY := (originalLeftAnkleDelta.FilledGlobalPosition().Y + originalRightAnkleDelta.FilledGlobalPosition().Y) / 2.0

			// sizingLeftAnkleDelta := sizingMorphAllDeltas[index].Bones.GetByName(pmx.ANKLE.Left())
			// sizingRightAnkleDelta := sizingMorphAllDeltas[index].Bones.GetByName(pmx.ANKLE.Right())
			// sizingAnkleY := (sizingLeftAnkleDelta.FilledGlobalPosition().Y + sizingRightAnkleDelta.FilledGlobalPosition().Y) / 2.0

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
			trunkRootYRatio := (originalBodyAxisLocalPosition.Y) / originalInitialBodyAxisY

			// 先の体軸の高さに、その比率をかける
			sizingBodyAxisY := sizingInitialBodyAxisY * trunkRootYRatio

			// 先の理想体軸
			sizingBodyAxisIdealLocalPosition := originalBodyAxisLocalPosition.Muled(moveScale)
			// ローカル位置のYを置き換え
			sizingBodyAxisIdealLocalPosition.Y = sizingBodyAxisY

			// 初期姿勢からの差分
			sizingLegCenterDiff := sizingBodyAxisIdealLocalPosition.Subed(sizingBodyAxisInitialLocalPosition)

			centerPositions[index] = sizingLegCenterDiff.Copy()

			if mlog.IsDebug() {
				recordDebugData(index, debugBoneNames, originalAllDeltas[index],
					debugTargetOriginal, debugTypeInitial, debugPositions, debugRotations)
				recordDebugData(index, debugBoneNames, sizingAllDeltas[index],
					debugTargetSizing, debugTypeInitial, debugPositions, debugRotations)

				// 先のセンター親
				sizingCenterParentDelta := sizingAllDeltas[index].Bones.GetByName(
					sizingSet.OriginalCenterBone().ParentBone.Name())
				debugPositions[debugTargetSizing][debugTypeResult][pmx.BODY_AXIS.String()][index] =
					sizingCenterParentDelta.FilledGlobalMatrix().MulVec3(sizingBodyAxisIdealLocalPosition)
			}

			if isActiveGroove {
				groovePositions[index] = &mmath.MVec3{X: 0.0, Y: sizingLegCenterDiff.Y, Z: 0.0}
				centerPositions[index].Y = 0.0
			}

			// 全親は単純なスケール
			if originalAllDeltas[index].Bones.ContainsByName(pmx.ROOT.String()) {
				originalRootDelta := originalAllDeltas[index].Bones.GetByName(pmx.ROOT.String())
				rootPositions[index] = originalRootDelta.FilledFramePosition().Muled(moveScale)
			}

			{
				// デフォーム情報を更新するため、クリア
				for _, boneName := range []pmx.StandardBoneName{
					pmx.ROOT, pmx.CENTER, pmx.GROOVE,
				} {
					sizingDelta := sizingAllDeltas[index].Bones.GetByName(boneName.String())
					if sizingDelta == nil {
						continue
					}
					sizingDelta.UnitMatrix = nil
					sizingAllDeltas[index].Bones.Update(sizingDelta)
				}
			}

			incrementCompletedCount()

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正05", sizingSet.Index, iterIndex, allCount)
		})
	if err != nil {
		return nil, nil, nil, err
	}

	if mlog.IsDebug() {
		outputDebugData(allFrames, debugBoneNames, debugMotionKey, sizingSet.OutputMotionPath, sizingSet.SizingConfigModel, debugPositions, debugRotations)
	}

	return rootPositions, centerPositions, groovePositions, nil
}

// updateCenter は、計算したセンター・グルーブ位置をサイジング先モーションに反映します。
func (su *SizingLegUsecase) updateCenter(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	rootPositions, centerPositions, groovePositions []*mmath.MVec3,
	incrementCompletedCount func(),
) {
	for i, iFrame := range allFrames {
		frame := float32(iFrame)

		for _, v := range [][]any{
			{pmx.ROOT.String(), rootPositions[i]},
			{pmx.CENTER.String(), centerPositions[i]},
			{pmx.GROOVE.String(), groovePositions[i]},
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
	tailBone, _ := sizingSet.SizingConfigModel.Bones.GetAnkleD(direction)
	ikBone = pmx.NewBoneByName(fmt.Sprintf("%s%sIk", pmx.MLIB_PREFIX, tailBone.Name()))

	ikBone.Position = tailBone.Position.Copy()
	ikBone.Ik = pmx.NewIk()
	ikBone.Ik.BoneIndex = tailBone.Index()
	ikBone.Ik.LoopCount = 1000
	ikBone.Ik.UnitRotation = &mmath.MVec3{X: 0.01, Y: 0.0, Z: 0.0}
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
	}

	return ikBone
}

// calculateAdjustedLegIK は、足IK 補正の計算を並列処理で行い、各フレームごとの位置・回転補正値を算出します。
func (su *SizingLegUsecase) calculateAdjustedLegIK2(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, moveScale *mmath.MVec3,
	originalAllDeltas, sizingLegIkOffAllDeltas, sizingLegIkOnAllDeltas,
	originalMorphAllDeltas, sizingMorphAllDeltas []*delta.VmdDeltas,
	sizingProcessMotion *vmd.VmdMotion, incrementCompletedCount func(), verboseMotionKey string,
) (legIkPositions [][]*mmath.MVec3,
	legIkRotations, legRotations, kneeRotations, ankleRotations [][]*mmath.MQuaternion,
	err error) {
	legIkPositions = make([][]*mmath.MVec3, 2)
	legIkRotations = make([][]*mmath.MQuaternion, 2)
	legRotations = make([][]*mmath.MQuaternion, 2)
	kneeRotations = make([][]*mmath.MQuaternion, 2)
	ankleRotations = make([][]*mmath.MQuaternion, 2)

	for d := range directions {
		legIkPositions[d] = make([]*mmath.MVec3, len(allFrames))
		legIkRotations[d] = make([]*mmath.MQuaternion, len(allFrames))
		legRotations[d] = make([]*mmath.MQuaternion, len(allFrames))
		kneeRotations[d] = make([]*mmath.MQuaternion, len(allFrames))
		ankleRotations[d] = make([]*mmath.MQuaternion, len(allFrames))
	}

	toeIkTargetBones := make([]*pmx.Bone, 2)
	toeIkTargetBones[0], _ = sizingSet.SizingConfigModel.Bones.Get(
		sizingSet.SizingToeIkBone(pmx.BONE_DIRECTION_LEFT).Ik.BoneIndex)
	toeIkTargetBones[1], _ = sizingSet.SizingConfigModel.Bones.Get(
		sizingSet.SizingToeIkBone(pmx.BONE_DIRECTION_RIGHT).Ik.BoneIndex)

	ankleIkBones := make([]*pmx.Bone, 2)
	ankleIkBones[0] = su.createFullAnkleIkBone(sizingSet, pmx.BONE_DIRECTION_LEFT)
	ankleIkBones[1] = su.createFullAnkleIkBone(sizingSet, pmx.BONE_DIRECTION_RIGHT)

	debugBoneNames := []pmx.StandardBoneName{
		pmx.ANKLE_D, pmx.TOE_T_D, pmx.HEEL_D,
	}
	debugPositions, debugRotations := newDebugData(allFrames, debugBoneNames)

	err = miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.NewTerminateError("manual terminate")
			}

			for d, direction := range directions {
				// 足根元から足首地面までの長さ差
				originalAnkleLength := originalMorphAllDeltas[index].Bones.GetByName(
					pmx.ANKLE.StringFromDirection(direction)).FilledGlobalPosition().Y
				sizingAnkleLength := sizingMorphAllDeltas[index].Bones.GetByName(
					pmx.ANKLE.StringFromDirection(direction)).FilledGlobalPosition().Y
				ankleScale := sizingAnkleLength / originalAnkleLength

				// --------------------------------

				// 足IKがターゲットより伸びている場合の対応
				originalLegIkDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG_IK.StringFromDirection(direction))
				originalAnkleDelta := originalAllDeltas[index].Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))
				originalLegIkDiff := originalLegIkDelta.FilledGlobalPosition().Subed(originalAnkleDelta.FilledGlobalPosition())
				// 先モデルにおける足首から見た足IKの差分を求める
				sizingLegIkDiff := originalLegIkDiff.Muled(moveScale)

				ankleDelta := sizingLegIkOffAllDeltas[index].Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))

				sizingAnkleIdealGlobalPosition := ankleDelta.FilledGlobalPosition().Added(sizingLegIkDiff)
				sizingAnkleIdealGlobalPosition.Y += su.calculateAnkleYDiff(originalMorphAllDeltas[index],
					originalAllDeltas[index], sizingLegIkOffAllDeltas[index], direction, ankleScale)

				toeTargetDelta := sizingLegIkOnAllDeltas[index].Bones.Get(toeIkTargetBones[d].Index())

				ankleYDiff := sizingAnkleIdealGlobalPosition.Y - ankleDelta.FilledGlobalPosition().Y
				sizingToeIdealGlobalPosition := toeTargetDelta.FilledGlobalPosition().Added(sizingLegIkDiff)
				sizingToeIdealGlobalPosition.Y += ankleYDiff

				if mlog.IsDebug() {
					recordDebugData(index, debugBoneNames, originalAllDeltas[index],
						debugTargetOriginal, debugTypeInitial, debugPositions, debugRotations)
					recordDebugData(index, debugBoneNames, sizingLegIkOffAllDeltas[index],
						debugTargetSizing, debugTypeInitial, debugPositions, debugRotations)

					debugPositions[debugTargetSizing][debugTypeIdeal][pmx.ANKLE_D.StringFromDirection(direction)][index] = sizingAnkleIdealGlobalPosition.Copy()
					debugPositions[debugTargetSizing][debugTypeIdeal][pmx.TOE_T_D.StringFromDirection(direction)][index] = sizingToeIdealGlobalPosition.Copy()
				}

				// IK解決
				sizingLegDeltas, _ := deform.DeformIks(sizingSet.SizingConfigModel, sizingProcessMotion,
					sizingLegIkOnAllDeltas[index], float32(data),
					[]*pmx.Bone{ankleIkBones[d], sizingSet.SizingToeIkBone(direction)},
					[]*pmx.Bone{sizingSet.SizingAnkleBone(direction), toeIkTargetBones[d]},
					[]*mmath.MVec3{sizingAnkleIdealGlobalPosition, sizingToeIdealGlobalPosition},
					leg_direction_bone_names[d], 1, false, false)

				// 足IKの親からみた足IKデフォルト位置からみた現在の足首のローカル位置
				{
					legIkParentBoneIndex := sizingSet.SizingLegIkBone(direction).ParentBone.Index()
					sizingLegIkMorphDelta := sizingMorphAllDeltas[index].Bones.Get(sizingSet.SizingLegIkBone(direction).Index())
					sizingLegIkParentMorphDelta := sizingMorphAllDeltas[index].Bones.Get(legIkParentBoneIndex)
					sizingLegIkMorphLocalPosition :=
						sizingLegIkParentMorphDelta.FilledGlobalMatrix().Inverted().MulVec3(
							sizingLegIkMorphDelta.FilledGlobalPosition())

					sizingAnkleDelta := sizingLegDeltas.Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))
					sizingLegIkParentDelta := sizingLegDeltas.Bones.Get(legIkParentBoneIndex)
					legIkPositions[d][index] = sizingLegIkParentDelta.FilledGlobalMatrix().Inverted().MulVec3(sizingAnkleDelta.FilledGlobalPosition()).Subed(sizingLegIkMorphLocalPosition)
					legIkRotations[d][index] = sizingLegDeltas.Bones.GetByName(pmx.LEG_IK.StringFromDirection(direction)).FilledFrameRotation()
				}

				legRotations[d][index] = sizingLegDeltas.Bones.GetByName(pmx.LEG.StringFromDirection(direction)).FilledFrameRotation().Copy()
				kneeRotations[d][index] = sizingLegDeltas.Bones.GetByName(pmx.KNEE.StringFromDirection(direction)).FilledFrameRotation().Copy()
				ankleRotations[d][index] = sizingLegDeltas.Bones.GetByName(pmx.ANKLE.StringFromDirection(direction)).FilledFrameRotation().Copy()

				if mlog.IsDebug() {
					recordDebugData(index, debugBoneNames, sizingLegDeltas,
						debugTargetSizing, debugTypeResult, debugPositions, debugRotations)
				}

				{
					// デフォーム情報を更新するため、クリア
					for _, boneName := range []pmx.StandardBoneName{
						pmx.LEG_IK_PARENT, pmx.LEG_IK, pmx.LEG, pmx.KNEE, pmx.ANKLE,
					} {
						sizingDelta := sizingLegIkOnAllDeltas[index].Bones.GetByName(boneName.StringFromDirection(direction))
						if sizingDelta == nil {
							continue
						}
						sizingDelta.UnitMatrix = nil
						sizingLegIkOnAllDeltas[index].Bones.Update(sizingDelta)
					}
				}
			}

			incrementCompletedCount()

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正07", sizingSet.Index, iterIndex, allCount)
		})
	if err != nil {
		return nil, nil, nil, nil, nil, err
	}

	if mlog.IsDebug() {
		outputDebugData(allFrames, debugBoneNames, verboseMotionKey, sizingSet.OutputMotionPath, sizingSet.SizingConfigModel, debugPositions, debugRotations)
	}

	return legIkPositions, legIkRotations, legRotations, kneeRotations, ankleRotations, nil
}

func (su *SizingLegUsecase) calculateAnkleYDiff(
	originalMorphDelta, originalDelta, sizingDelta *delta.VmdDeltas, direction pmx.BoneDirection, ankleScale float64,
) float64 {
	originalMorphAnkleDelta := originalMorphDelta.Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))
	// originalAnkleDelta := originalDelta.Bones.GetByName(pmx.LEG_IK.StringFromDirection(direction))
	originalToeTailDelta := originalDelta.Bones.GetByName(pmx.TOE_T_D.StringFromDirection(direction))
	originalHeelDelta := originalDelta.Bones.GetByName(pmx.HEEL_D.StringFromDirection(direction))
	sizingToeTailDelta := sizingDelta.Bones.GetByName(pmx.TOE_T_D.StringFromDirection(direction))
	sizingHeelDelta := sizingDelta.Bones.GetByName(pmx.HEEL_D.StringFromDirection(direction))
	// sizingAnkleDelta := sizingDelta.Bones.GetByName(pmx.ANKLE_D.StringFromDirection(direction))

	originalMorphAnkleY := originalMorphAnkleDelta.FilledGlobalPosition().Y
	originalToeTailDY := originalToeTailDelta.FilledGlobalPosition().Y
	originalHeelDY := originalHeelDelta.FilledGlobalPosition().Y

	// originalAnkleY := originalAnkleDelta.FilledGlobalPosition().Y

	// つま先の補正値 ------------------
	// つま先のY座標を元モデルのつま先のY座標*スケールに合わせる
	idealSizingToeTailY := originalToeTailDY * ankleScale

	// 現時点のつま先のY座標
	actualToeTailY := sizingToeTailDelta.FilledGlobalPosition().Y

	toeDiff := idealSizingToeTailY - actualToeTailY
	lerpToeDiff := mmath.Lerp(toeDiff, 0, originalToeTailDY/originalMorphAnkleY)

	// かかとの補正値 ------------------
	// かかとのY座標を元モデルのかかとのY座標*スケールに合わせる
	idealSizingHeelY := originalHeelDY * ankleScale

	// 現時点のかかとのY座標
	actualSizingHeelY := sizingHeelDelta.FilledGlobalPosition().Y

	heelDiff := idealSizingHeelY - actualSizingHeelY
	lerpHeelDiff := mmath.Lerp(heelDiff, 0, originalHeelDY/originalMorphAnkleY)

	// 最終的な差分(つま先のY位置が地面に近いほど、つま先側を優先採用)
	diffAnkleYByTail := mmath.Lerp(lerpToeDiff, lerpHeelDiff, originalToeTailDY/originalHeelDY)

	// // 足IKの補正値 ------------------
	// // 足IKの位置が0に近い場合、そちらに寄せる
	// idealLegIkY := originalAnkleY * ankleScale

	// // 現時点の足IKのY座標
	// actualSizingAnkleY := sizingAnkleDelta.FilledGlobalPosition().Y
	// legIkDiff := idealLegIkY - actualSizingAnkleY

	// lerpLegIkDiff := mmath.Lerp(legIkDiff, 0, originalAnkleY/originalMorphAnkleY)

	// diffAnkleYByLegIk := mmath.Lerp(lerpLegIkDiff, diffAnkleYByTail, originalAnkleY/originalMorphAnkleY-1.0)

	return diffAnkleYByTail
}

// updateLegIkAndFk2 は、計算済みの足IK 補正位置と回転値をモーションデータに反映させます。
func (su *SizingLegUsecase) updateLegIkAndFk2(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	legIkPositions [][]*mmath.MVec3,
	legIkRotations, legRotations, kneeRotations, ankleRotations [][]*mmath.MQuaternion,
	incrementCompletedCount func(),
) {
	emptyPositions := make([][]*mmath.MVec3, 2)
	emptyPositions[0] = make([]*mmath.MVec3, len(allFrames))
	emptyPositions[1] = make([]*mmath.MVec3, len(allFrames))

	for i, iFrame := range allFrames {
		frame := float32(iFrame)

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
	legScale float64, verboseMotionKey string, incrementCompletedCount func(),
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
	intervalFrames := mmath.IntRangesByStep(allFrames[0], allFrames[len(allFrames)-1], 4)

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
			outputMotion.InsertBoneFrame(boneName, bf)
			return true
		})
	}

	if mlog.IsDebug() {
		outputVerboseMotion(verboseMotionKey, sizingSet.OutputMotionPath, outputMotion)
	}

	// 中間キーフレのズレをチェック
	legThreshold := 0.2 * legScale
	kneeThreshold := 0.4 * legScale
	ankleThreshold := 0.3 * legScale
	heelThreshold := 0.3 * legScale

	err := miter.IterParallelByList(directions, 1, 1,
		func(dIndex int, direction pmx.BoneDirection) error {
			for tIndex, targetFrames := range [][]int{activeFrames, intervalFrames, allFrames} {
				processAllDeltas, err := computeVmdDeltas(targetFrames, blockSize,
					sizingModel, sizingProcessMotion, sizingSet, true, all_lower_leg_bone_names, "足補正01", incrementCompletedCount)
				if err != nil {
					return err
				}

				prevLog := 0
				prevFrame := 0
				for fIndex, iFrame := range targetFrames {
					if sizingSet.IsTerminate {
						return merr.NewTerminateError("manual terminate")
					}
					frame := float32(iFrame)

					// 現時点の結果
					resultAllVmdDeltas, err := computeVmdDeltas([]int{iFrame}, 1,
						sizingModel, outputMotion, sizingSet, true, leg_direction_bone_names[dIndex], "", nil)
					if err != nil {
						return err
					}

					// 足の位置をチェック
					resultLegDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.LEG_D.StringFromDirection(direction))
					processLegDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.LEG_D.StringFromDirection(direction))

					// ひざの位置をチェック
					resultKneeDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.KNEE_D.StringFromDirection(direction))
					processKneeDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.KNEE_D.StringFromDirection(direction))

					// 足首の位置をチェック
					resultAnkleDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.ANKLE_D.StringFromDirection(direction))
					processAnkleDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.ANKLE_D.StringFromDirection(direction))

					// かかとの位置をチェック
					resultHeelDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.HEEL_D.StringFromDirection(direction))
					processHeelDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.HEEL_D.StringFromDirection(direction))

					if resultLegDelta.FilledGlobalPosition().Distance(processLegDelta.FilledGlobalPosition()) > legThreshold ||
						resultKneeDelta.FilledGlobalPosition().Distance(processKneeDelta.FilledGlobalPosition()) > kneeThreshold ||
						resultAnkleDelta.FilledGlobalPosition().Distance(processAnkleDelta.FilledGlobalPosition()) > ankleThreshold ||
						resultHeelDelta.FilledGlobalPosition().Distance(processHeelDelta.FilledGlobalPosition()) > heelThreshold {

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

func (su *SizingLegUsecase) updateLegIkOffset(sizingSet *domain.SizingSet, allFrames []int, legScale float64) {
	// 元モーションのIKが動いていない区間を取得
	fixIkFlags := make([][]bool, len(directions))
	fixIkFlags[0] = make([]bool, len(allFrames))
	fixIkFlags[1] = make([]bool, len(allFrames))

	for i, iFrame := range allFrames {
		for d, direction := range directions {
			boneName := pmx.LEG_IK.StringFromDirection(direction)

			if i == 0 {
				fixIkFlags[d][i] = false
				continue
			}

			frame := float32(iFrame)
			bf := sizingSet.OriginalMotion.BoneFrames.Get(boneName).Get(frame)
			prevBf := sizingSet.OriginalMotion.BoneFrames.Get(boneName).Get(frame - 1)
			fixIkFlags[d][i] = bf.FilledPosition().NearEquals(prevBf.FilledPosition(), 1e-2*legScale)
		}

		if i > 0 && i%1000 == 0 {
			processLog("足補正12", sizingSet.Index, i, len(allFrames))
		}
	}

	processLog("足補正13", sizingSet.Index, 0, 0)

	for d, direction := range directions {
		{
			boneName := pmx.LEG_IK.StringFromDirection(direction)
			sizingSet.OutputMotion.BoneFrames.Get(boneName).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
				if fixIkFlags[d][int(frame)] {
					// 固定されている場合、前のキーフレを引き継ぐ
					prevFrame := sizingSet.OutputMotion.BoneFrames.Get(boneName).PrevFrame(frame)
					prevBf := sizingSet.OutputMotion.BoneFrames.Get(boneName).Get(prevFrame)
					bf.Position = prevBf.FilledPosition().Copy()
				}
				// originalBf := sizingSet.OriginalMotion.BoneFrames.Get(boneName).Get(frame)
				// if mmath.NearEquals(originalBf.FilledPosition().Y, 0.0, 1e-2) {
				// 	// 元の足IKが0の場合、0にする
				// 	bf.Position.Y = 0.0
				// }

				sizingSet.OutputMotion.BoneFrames.Get(boneName).Update(bf)

				return true
			})
		}
		{
			// あえて足ボーンを少し動かす
			boneName := pmx.LEG.StringFromDirection(direction)
			offsetQUat := mmath.NewMQuaternionFromDegrees(1, 0, 0)
			sizingSet.OutputMotion.BoneFrames.Get(boneName).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
				bf.Rotation.Mul(offsetQUat)
				sizingSet.OutputMotion.BoneFrames.Get(boneName).Update(bf)
				return true
			})
		}
	}
}

func (su *SizingLegUsecase) createLowerBoneNames(sizingSet *domain.SizingSet) []string {
	leftToeIkTargetBone, _ := sizingSet.SizingConfigModel.Bones.Get(
		sizingSet.SizingToeIkBone(pmx.BONE_DIRECTION_LEFT).Ik.BoneIndex)
	rightToeIkTargetBone, _ := sizingSet.SizingConfigModel.Bones.Get(
		sizingSet.SizingToeIkBone(pmx.BONE_DIRECTION_RIGHT).Ik.BoneIndex)

	return append(all_lower_leg_bone_names, leftToeIkTargetBone.Name(), rightToeIkTargetBone.Name())
}

func (su *SizingLegUsecase) checkBones(sizingSet *domain.SizingSet) (err error) {
	// グルーブはサイジング先に元々存在している場合のみ取得
	sizingSet.SizingGrooveVanillaBone()

	return checkBones(
		sizingSet,
		[]domain.CheckTrunkBoneType{
			{CheckFunk: sizingSet.OriginalCenterBone, BoneName: pmx.CENTER},
			{CheckFunk: sizingSet.OriginalLowerBone, BoneName: pmx.LOWER},
			{CheckFunk: sizingSet.OriginalBodyAxisBone, BoneName: pmx.BODY_AXIS},
			{CheckFunk: sizingSet.OriginalLegCenterBone, BoneName: pmx.LEG_CENTER},
			{CheckFunk: sizingSet.OriginalTrunkRootBone, BoneName: pmx.TRUNK_ROOT},
		},
		[]domain.CheckDirectionBoneType{
			{CheckFunk: sizingSet.OriginalArmBone, BoneName: pmx.ARM},
			{CheckFunk: sizingSet.OriginalLegIkBone, BoneName: pmx.LEG_IK},
			{CheckFunk: sizingSet.OriginalLegRootBone, BoneName: pmx.LEG_ROOT},
			{CheckFunk: sizingSet.OriginalLegBone, BoneName: pmx.LEG},
			{CheckFunk: sizingSet.OriginalKneeBone, BoneName: pmx.KNEE},
			{CheckFunk: sizingSet.OriginalKneeDBone, BoneName: pmx.KNEE_D},
			{CheckFunk: sizingSet.OriginalAnkleBone, BoneName: pmx.ANKLE},
			{CheckFunk: sizingSet.OriginalAnkleDBone, BoneName: pmx.ANKLE_D},
			{CheckFunk: sizingSet.OriginalAnkleDGroundBone, BoneName: pmx.ANKLE_D_GROUND},
			{CheckFunk: sizingSet.OriginalToeIkBone, BoneName: pmx.TOE_IK},
			{CheckFunk: sizingSet.OriginalHeelDBone, BoneName: pmx.HEEL_D},
			{CheckFunk: sizingSet.OriginalToeTailDBone, BoneName: pmx.TOE_T_D},
			{CheckFunk: sizingSet.OriginalLegIkParentBone, BoneName: pmx.LEG_IK_PARENT},
		},
		[]domain.CheckTrunkBoneType{
			{CheckFunk: sizingSet.SizingCenterBone, BoneName: pmx.CENTER},
			{CheckFunk: sizingSet.SizingLowerBone, BoneName: pmx.LOWER},
			{CheckFunk: sizingSet.SizingBodyAxisBone, BoneName: pmx.BODY_AXIS},
			{CheckFunk: sizingSet.SizingLegCenterBone, BoneName: pmx.LEG_CENTER},
			{CheckFunk: sizingSet.SizingTrunkRootBone, BoneName: pmx.TRUNK_ROOT},
		},
		[]domain.CheckDirectionBoneType{
			{CheckFunk: sizingSet.SizingArmBone, BoneName: pmx.ARM},
			{CheckFunk: sizingSet.SizingLegIkBone, BoneName: pmx.LEG_IK},
			{CheckFunk: sizingSet.SizingLegRootBone, BoneName: pmx.LEG_ROOT},
			{CheckFunk: sizingSet.SizingLegBone, BoneName: pmx.LEG},
			{CheckFunk: sizingSet.SizingKneeBone, BoneName: pmx.KNEE},
			{CheckFunk: sizingSet.SizingKneeDBone, BoneName: pmx.KNEE_D},
			{CheckFunk: sizingSet.SizingAnkleBone, BoneName: pmx.ANKLE},
			{CheckFunk: sizingSet.SizingAnkleDBone, BoneName: pmx.ANKLE_D},
			{CheckFunk: sizingSet.SizingAnkleDGroundBone, BoneName: pmx.ANKLE_D_GROUND},
			{CheckFunk: sizingSet.SizingToeIkBone, BoneName: pmx.TOE_IK},
			{CheckFunk: sizingSet.SizingHeelDBone, BoneName: pmx.HEEL_D},
			{CheckFunk: sizingSet.SizingToeTailDBone, BoneName: pmx.TOE_T_D},
			{CheckFunk: sizingSet.SizingLegIkParentBone, BoneName: pmx.LEG_IK_PARENT},
		},
	)
}
