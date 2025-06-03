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
		// エラーは上には返さない
		return false, nil
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
	lowerRotations, err := su.calculateAdjustedLower(sizingSet, allFrames, blockSize, moveScale,
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

	// 足IK親 キーフレームのリセット
	sizingProcessMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.LEG_IK_PARENT.Left()))
	sizingProcessMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.LEG_IK_PARENT.Right()))

	// 先モデルの足のIK OFF状態でのデフォーム結果を並列処理で取得
	sizingAllDeltas, err = computeVmdDeltasWithDeltas(allFrames, blockSize, sizingSet.SizingConfigModel,
		sizingProcessMotion, sizingAllDeltas, sizingSet, false, lowerBoneNames, "足補正01", incrementCompletedCount)
	if err != nil {
		return false, err
	}

	// 足IK 補正処理
	legIkPositions, legIkRotations, legRotations, ankleRotations, err :=
		su.calculateAdjustedLegIk(sizingSet, allFrames, blockSize, moveScale,
			originalAllDeltas, sizingAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas,
			sizingProcessMotion, incrementCompletedCount, "足04")
	if err != nil {
		return false, err
	}

	// 足IKの位置と足の角度 をサイジング先モーションに反映
	su.updateLegIkAndFk(sizingSet, allFrames, sizingProcessMotion,
		legIkPositions, legIkRotations, legRotations, nil, ankleRotations, nil, incrementCompletedCount)

	// TOE_IK キーフレームのリセット
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
	rootPositions, centerPositions, groovePositions, isActiveGroove, err :=
		su.calculateAdjustedCenter(sizingSet, allFrames, blockSize, moveScale,
			originalAllDeltas, sizingAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas, legIkPositions,
			sizingProcessMotion, incrementCompletedCount, "足06")
	if err != nil {
		return false, err
	}

	// センター・グルーブ位置をサイジング先モーションに反映
	su.updateCenter(sizingSet, allFrames, sizingProcessMotion,
		rootPositions, centerPositions, groovePositions,
		incrementCompletedCount)

	// 足IKの位置 をサイジング先モーションに反映
	su.updateLegIkAndFk(sizingSet, allFrames, sizingProcessMotion,
		legIkPositions, legIkRotations, nil, nil, nil, nil, incrementCompletedCount)

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

	// IKをONに戻す
	su.insertIKFrames(sizingSet, sizingProcessMotion, true)

	// 先モデルの足FK補正デフォーム結果を並列処理で取得
	sizingLegIkOnAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel,
		sizingProcessMotion, sizingSet, true, lowerBoneNames, "足補正01", incrementCompletedCount)
	if err != nil {
		return false, err
	}

	// 足IK 補正処理
	legIkPositions, legIkRotations, legRotations, kneeRotations, ankleRotations, toeExRotations, err :=
		su.calculateAdjustedLegIk2(sizingSet, allFrames, blockSize, moveScale,
			originalAllDeltas, sizingLegIkOffAllDeltas, sizingLegIkOnAllDeltas,
			originalMorphAllDeltas, sizingMorphAllDeltas,
			sizingProcessMotion, incrementCompletedCount, "足08")
	if err != nil {
		return false, err
	}

	// 足IKの位置と足の角度 をサイジング先モーションに反映
	su.updateLegIkAndFk(sizingSet, allFrames, sizingProcessMotion, legIkPositions,
		legIkRotations, legRotations, kneeRotations, ankleRotations, toeExRotations, incrementCompletedCount)

	if mlog.IsDebug() {
		outputVerboseMotion("足09", sizingSet.OutputMotionPath, sizingProcessMotion)
	}

	// // [足IK回転] -----------------------

	// // 先モデルの足FK補正デフォーム結果を並列処理で取得
	// sizingAllDeltas, err = computeVmdDeltasWithDeltas(allFrames, blockSize, sizingSet.SizingConfigModel,
	// 	sizingProcessMotion, sizingLegIkOnAllDeltas, sizingSet, true, lowerBoneNames, "足補正01", incrementCompletedCount)
	// if err != nil {
	// 	return false, err
	// }

	// // センター・グルーブ補正を実施
	// legIkRotations, err =
	// 	su.calculateAdjustedLegIk3(sizingSet, allFrames, blockSize, sizingAllDeltas, incrementCompletedCount)
	// if err != nil {
	// 	return false, err
	// }

	// // 足IKの位置と足の角度 をサイジング先モーションに反映
	// su.updateLegIkAndFk(sizingSet, allFrames, sizingProcessMotion, nil,
	// 	legIkRotations, nil, nil, nil, incrementCompletedCount)

	// if mlog.IsDebug() {
	// 	outputVerboseMotion("足10", sizingSet.OutputMotionPath, sizingProcessMotion)
	// }

	// [結果出力] -----------------------

	// あにまさミクを基準に、近似値のスケールを求める
	legScale := sizingSet.SizingLegCenterBone().Position.Y / 10

	// 足補正処理の結果をサイジング先モーションに反映
	if err = su.updateOutputMotion(
		sizingSet, allFrames, blockSize, isActiveGroove, sizingProcessMotion, legScale, "足11",
		incrementCompletedCount,
	); err != nil {
		return false, err
	}

	if mlog.IsDebug() {
		outputVerboseMotion("足12", sizingSet.OutputMotionPath, sizingSet.OutputMotion)
	}

	// 足IKのオフセットを元に戻す
	su.updateLegIkOffset(sizingSet, originalAllDeltas, allFrames, legScale, "足13")

	if mlog.IsDebug() {
		outputVerboseMotion("足14", sizingSet.OutputMotionPath, sizingSet.OutputMotion)
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
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, moveScale *mmath.MVec3,
	originalAllDeltas, sizingAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas []*delta.VmdDeltas,
	sizingProcessMotion *vmd.VmdMotion, incrementCompletedCount func(), debugMotionKey string,
) (sizingLowerResultRotations []*mmath.MQuaternion, err error) {
	sizingLowerResultRotations = make([]*mmath.MQuaternion, len(allFrames))

	lowerIkBone := su.createLowerIkBone(sizingSet, pmx.BONE_DIRECTION_TRUNK)
	// leftLegIkBone := su.createLowerIkBone(sizingSet, pmx.BONE_DIRECTION_LEFT)
	// rightLegIkBone := su.createLowerIkBone(sizingSet, pmx.BONE_DIRECTION_RIGHT)

	debugBoneNames := []pmx.StandardBoneName{
		pmx.LOWER, pmx.LEG_CENTER, pmx.LEG, pmx.HIP,
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
			sizingMorphLegCenterRelativePosition := sizingMorphLegCenterDelta.FilledGlobalPosition().Subed(
				sizingMorphLowerDelta.FilledGlobalPosition())

			// 真下から足中心までの傾き
			originalLegSlope := mmath.NewMQuaternionRotate(mmath.MVec3UnitYNeg, originalMorphLegCenterRelativePosition.Normalized())
			sizingLegSlope := mmath.NewMQuaternionRotate(mmath.MVec3UnitYNeg, sizingMorphLegCenterRelativePosition.Normalized())
			sizingLegSlopeMat := sizingLegSlope.ToMat4()

			// 下半身の長さ
			originalLowerHeight := originalMorphLowerDelta.FilledGlobalPosition().Distance(originalMorphLegCenterDelta.FilledGlobalPosition())
			sizingLowerHeight := sizingMorphLowerDelta.FilledGlobalPosition().Distance(sizingMorphLegCenterDelta.FilledGlobalPosition())

			// 足幅
			originalLegWidth := originalMorphLeftLegDelta.FilledGlobalPosition().Distance(originalMorphRightLegDelta.FilledGlobalPosition())
			sizingLegWidth := sizingMorphLeftLegDelta.FilledGlobalPosition().Distance(sizingMorphRightLegDelta.FilledGlobalPosition())

			legCenterFromLowerScale := &mmath.MVec3{
				X: sizingLegWidth / originalLegWidth,
				Y: sizingLowerHeight / originalLowerHeight,
				Z: 1.0}

			// -------------------------

			originalLowerRootDelta := originalAllDeltas[index].Bones.GetByName(pmx.LOWER_ROOT.String())
			originalLowerDelta := originalAllDeltas[index].Bones.GetByName(pmx.LOWER.String())
			originalLegCenterDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG_CENTER.String())
			originalLeftLegDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG.Left())
			originalRightLegDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG.Right())

			// 下半身の軸回転を取得
			lowerTwistQuat, _ := originalLowerDelta.FilledFrameRotation().SeparateTwistByAxis(mmath.MVec3UnitYNeg)
			lowerTwistMat := lowerTwistQuat.ToMat4()

			// 下半身根元に下半身の軸回転を加えたところから見た足ボーンのローカル位置
			originalLegCenterLocalPosition := originalLowerRootDelta.FilledGlobalMatrix().Copy().Muled(
				lowerTwistMat).Inverted().MulVec3(originalLegCenterDelta.FilledGlobalPosition())
			originalLeftLegLocalPosition := originalLowerRootDelta.FilledGlobalMatrix().Copy().Muled(
				lowerTwistMat).Inverted().MulVec3(originalLeftLegDelta.FilledGlobalPosition())
			originalRightLegLocalPosition := originalLowerRootDelta.FilledGlobalMatrix().Copy().Muled(
				lowerTwistMat).Inverted().MulVec3(originalRightLegDelta.FilledGlobalPosition())

			// 真っ直ぐにしたときのローカル位置
			originalLegCenterVerticalLocalPosition := originalLegSlope.Inverted().MulVec3(originalLegCenterLocalPosition).Truncate(1e-3)
			originalLeftLegVerticalLocalPosition := originalLegSlope.Inverted().MulVec3(originalLeftLegLocalPosition).Truncate(1e-3)
			originalRightLegVerticalLocalPosition := originalLegSlope.Inverted().MulVec3(originalRightLegLocalPosition).Truncate(1e-3)

			// スケール差を考慮した先の足ボーンのローカル位置
			sizingLegCenterVerticalLocalPosition := originalLegCenterVerticalLocalPosition.Muled(legCenterFromLowerScale)
			sizingLeftLegVerticalLocalPosition := originalLeftLegVerticalLocalPosition.Muled(legCenterFromLowerScale)
			sizingRightLegVerticalLocalPosition := originalRightLegVerticalLocalPosition.Muled(legCenterFromLowerScale)

			sizingLowerRootDelta := sizingAllDeltas[index].Bones.GetByName(pmx.LOWER_ROOT.String())
			sizingLegCenterIdealGlobalPosition := sizingLowerRootDelta.FilledGlobalMatrix().Muled(
				lowerTwistMat).Muled(sizingLegSlopeMat).MulVec3(sizingLegCenterVerticalLocalPosition)
			sizingLeftLegIdealGlobalPosition := sizingLowerRootDelta.FilledGlobalMatrix().Muled(
				lowerTwistMat).Muled(sizingLegSlopeMat).MulVec3(sizingLeftLegVerticalLocalPosition)
			sizingRightLegIdealGlobalPosition := sizingLowerRootDelta.FilledGlobalMatrix().Muled(
				lowerTwistMat).Muled(sizingLegSlopeMat).MulVec3(sizingRightLegVerticalLocalPosition)

			if mlog.IsDebug() {
				recordDebugData(index, debugBoneNames, originalAllDeltas[index],
					debugTargetOriginal, debugTypeInitial, debugPositions, debugRotations)
				recordDebugData(index, debugBoneNames, sizingAllDeltas[index],
					debugTargetSizing, debugTypeInitial, debugPositions, debugRotations)

				debugPositions[debugTargetSizing][debugTypeIdeal][pmx.LEG_CENTER.String()][index] = sizingLegCenterIdealGlobalPosition.Copy()
				debugPositions[debugTargetSizing][debugTypeIdeal][pmx.LEG.Left()][index] = sizingLeftLegIdealGlobalPosition.Copy()
				debugPositions[debugTargetSizing][debugTypeIdeal][pmx.LEG.Right()][index] = sizingRightLegIdealGlobalPosition.Copy()
			}

			// IK解決
			sizingLowerDeltas, _ := deform.DeformIks(sizingSet.SizingConfigModel, sizingProcessMotion,
				sizingAllDeltas[index], frame,
				[]*pmx.Bone{lowerIkBone},
				[]*pmx.Bone{sizingSet.SizingLegCenterBone()},
				[]*mmath.MVec3{sizingLegCenterIdealGlobalPosition},
				trunk_lower_bone_names, 0.1*moveScale.X, false, false)

			sizingLowerResultDelta := sizingLowerDeltas.Bones.GetByName(pmx.LOWER.String())
			sizingLowerResultRotations[index] = sizingLowerResultDelta.FilledFrameRotation().Copy()

			if mlog.IsDebug() {
				recordDebugData(index, debugBoneNames, sizingLowerDeltas,
					debugTargetSizing, debugTypeResult, debugPositions, debugRotations)
			}

			{
				// デフォーム情報を更新するため、クリア
				sizingLowerDelta := sizingAllDeltas[index].Bones.GetByName(pmx.LOWER.String())
				sizingAllDeltas[index].Bones.Delete(sizingLowerDelta.Bone.Index())
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

func (su *SizingLegUsecase) createKneeIkBone(sizingSet *domain.SizingSet, direction pmx.BoneDirection, tailBone *pmx.Bone) (
	ikBone *pmx.Bone,
) {
	legBone := sizingSet.SizingLegBone(direction)
	ikBone = pmx.NewBoneByName(fmt.Sprintf("%s%sIk", pmx.MLIB_PREFIX, tailBone.Name()))

	ikBone.Position = tailBone.Position.Copy()
	ikBone.Ik = pmx.NewIk()
	ikBone.Ik.BoneIndex = tailBone.Index()
	ikBone.Ik.LoopCount = 1000
	ikBone.Ik.UnitRotation = &mmath.MVec3{X: 0.01, Y: 0.0, Z: 0.0}
	ikBone.Ik.Links = make([]*pmx.IkLink, 0)

	for _, boneName := range []string{
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

// calculateAdjustedLegIk は、足IK 補正の計算を並列処理で行い、各フレームごとの位置・回転補正値を算出します。
func (su *SizingLegUsecase) calculateAdjustedLegIk(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, moveScale *mmath.MVec3,
	originalAllDeltas, sizingAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas []*delta.VmdDeltas,
	sizingProcessMotion *vmd.VmdMotion, incrementCompletedCount func(), verboseMotionKey string,
) (legIkPositions [][]*mmath.MVec3,
	legIkRotations, legRotations, ankleRotations [][]*mmath.MQuaternion, err error) {
	legIkPositions = make([][]*mmath.MVec3, 2)
	legIkRotations = make([][]*mmath.MQuaternion, 2)
	legRotations = make([][]*mmath.MQuaternion, 2)
	ankleRotations = make([][]*mmath.MQuaternion, 2)

	for i := range directions {
		legIkPositions[i] = make([]*mmath.MVec3, len(allFrames))
		legIkRotations[i] = make([]*mmath.MQuaternion, len(allFrames))
		legRotations[i] = make([]*mmath.MQuaternion, len(allFrames))
		ankleRotations[i] = make([]*mmath.MQuaternion, len(allFrames))
	}

	kneeIkBones := make([]*pmx.Bone, 2)
	kneeIkBones[0] = su.createKneeIkBone(
		sizingSet, pmx.BONE_DIRECTION_LEFT, sizingSet.SizingKneeBone(pmx.BONE_DIRECTION_LEFT))
	kneeIkBones[1] = su.createKneeIkBone(
		sizingSet, pmx.BONE_DIRECTION_RIGHT, sizingSet.SizingKneeBone(pmx.BONE_DIRECTION_RIGHT))

	ankleIkBones := make([]*pmx.Bone, 2)
	ankleIkBones[0] = su.createKneeIkBone(
		sizingSet, pmx.BONE_DIRECTION_LEFT, sizingSet.SizingAnkleBone(pmx.BONE_DIRECTION_LEFT))
	ankleIkBones[1] = su.createKneeIkBone(
		sizingSet, pmx.BONE_DIRECTION_RIGHT, sizingSet.SizingAnkleBone(pmx.BONE_DIRECTION_RIGHT))

	toeIkTargetBones := make([]*pmx.Bone, 2)
	toeIkTargetBones[0], _ = sizingSet.SizingConfigModel.Bones.Get(
		sizingSet.SizingToeIkBone(pmx.BONE_DIRECTION_LEFT).Ik.BoneIndex)
	toeIkTargetBones[1], _ = sizingSet.SizingConfigModel.Bones.Get(
		sizingSet.SizingToeIkBone(pmx.BONE_DIRECTION_RIGHT).Ik.BoneIndex)

	debugBoneNames := []pmx.StandardBoneName{
		pmx.LEG_CENTER, pmx.LEG, pmx.KNEE, pmx.ANKLE, pmx.LEG_IK, pmx.TOE_IK,
	}
	debugPositions, debugRotations := newDebugData(allFrames, debugBoneNames)

	err = miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, iFrame int) error {
			if sizingSet.IsTerminate {
				return merr.NewTerminateError("manual terminate")
			}

			for d, direction := range directions {
				// 腰骨から膝までの長さ差
				// originalMorphHipDelta := originalMorphAllDeltas[index].Bones.GetByName(
				// 	pmx.HIP.StringFromDirection(direction))
				originalMorphLegDelta := originalMorphAllDeltas[index].Bones.GetByName(
					pmx.LEG.StringFromDirection(direction))
				originalMorphKneeDelta := originalMorphAllDeltas[index].Bones.GetByName(
					pmx.KNEE.StringFromDirection(direction))
				originalMorphAnkleDelta := originalMorphAllDeltas[index].Bones.GetByName(
					pmx.ANKLE.StringFromDirection(direction))
				// sizingMorphHipDelta := sizingMorphAllDeltas[index].Bones.GetByName(
				// 	pmx.HIP.StringFromDirection(direction))
				sizingMorphLegDelta := sizingMorphAllDeltas[index].Bones.GetByName(
					pmx.LEG.StringFromDirection(direction))
				sizingMorphKneeDelta := sizingMorphAllDeltas[index].Bones.GetByName(
					pmx.KNEE.StringFromDirection(direction))
				sizingMorphAnkleDelta := sizingMorphAllDeltas[index].Bones.GetByName(
					pmx.ANKLE.StringFromDirection(direction))

				// originalLegLength := originalMorphHipDelta.FilledGlobalPosition().Distance(
				// 	originalMorphLegDelta.FilledGlobalPosition())
				// sizingLegLength := sizingMorphHipDelta.FilledGlobalPosition().Distance(
				// 	sizingMorphLegDelta.FilledGlobalPosition())

				originalKneeLength := originalMorphLegDelta.FilledGlobalPosition().Distance(
					originalMorphKneeDelta.FilledGlobalPosition())
				sizingKneeLength := sizingMorphLegDelta.FilledGlobalPosition().Distance(
					sizingMorphKneeDelta.FilledGlobalPosition())

				originalAnkleLength := originalMorphKneeDelta.FilledGlobalPosition().Distance(
					originalMorphAnkleDelta.FilledGlobalPosition())
				sizingAnkleLength := sizingMorphKneeDelta.FilledGlobalPosition().Distance(
					sizingMorphAnkleDelta.FilledGlobalPosition())

				// // 足の長さの比率
				// sizingLegScale := sizingLegLength / originalLegLength
				// // 足の長さの比率と同じ場合の腰骨の長さ
				// sizingIdealHipLength := originalHipLength * sizingLegScale
				// // 想定腰骨の長さと、実際の腰骨の長さの比率
				// sizingHipScale := sizingHipLength / sizingIdealHipLength

				// sizingHip2LegScale := sizingLegLength / originalLegLength
				sizingLeg2KneeScale := sizingKneeLength / originalKneeLength
				sizingKnee2AnkleScale := sizingAnkleLength / originalAnkleLength

				// -------------------------------

				// originalHipDelta := originalAllDeltas[index].Bones.GetByName(
				// 	pmx.HIP.StringFromDirection(direction))
				originalLegDelta := originalAllDeltas[index].Bones.GetByName(
					pmx.LEG.StringFromDirection(direction))
				originalKneeDelta := originalAllDeltas[index].Bones.GetByName(
					pmx.KNEE.StringFromDirection(direction))

				// sizingHipDelta := sizingAllDeltas[index].Bones.GetByName(
				// 	pmx.HIP.StringFromDirection(direction))
				sizingLegDelta := sizingAllDeltas[index].Bones.GetByName(
					pmx.LEG.StringFromDirection(direction))

				// // 腰骨ボーンから見た足までの相対位置
				// originalLegLocalPosition := originalLegDelta.FilledGlobalPosition().Subed(
				// 	originalHipDelta.FilledGlobalPosition())

				// // スケール差を考慮した先のひざボーンのローカル位置
				// sizingLegLocalPosition := originalLegLocalPosition.MuledScalar(sizingHip2LegScale)

				// sizingLegIdealGlobalPosition := sizingHipDelta.FilledGlobalPosition().Added(sizingLegLocalPosition)

				// 腰骨ボーンから見たひざまでの相対位置
				originalKneeLocalPosition := originalKneeDelta.FilledGlobalPosition().Subed(
					originalLegDelta.FilledGlobalPosition())

				// スケール差を考慮した先のひざボーンのローカル位置
				sizingKneeLocalPosition := originalKneeLocalPosition.MuledScalar(sizingLeg2KneeScale)

				sizingKneeIdealGlobalPosition := sizingLegDelta.FilledGlobalPosition().Added(sizingKneeLocalPosition)

				// ----------------------------------

				originalAnkleDelta := originalAllDeltas[index].Bones.GetByName(
					pmx.ANKLE.StringFromDirection(direction))

				// ひざボーンから見た足首までの相対位置
				originalAnkleLocalPosition := originalAnkleDelta.FilledGlobalPosition().Subed(
					originalKneeDelta.FilledGlobalPosition())

				// スケール差を考慮した先の足首ボーンのローカル位置
				sizingAnkleLocalPosition := originalAnkleLocalPosition.MuledScalar(sizingKnee2AnkleScale)

				sizingAnkleIdealGlobalPosition := sizingKneeIdealGlobalPosition.Added(sizingAnkleLocalPosition)

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

					// debugPositions[debugTargetSizing][debugTypeIdeal][pmx.LEG.StringFromDirection(direction)][index] = sizingLegIdealGlobalPosition.Copy()
					debugPositions[debugTargetSizing][debugTypeIdeal][pmx.KNEE.StringFromDirection(direction)][index] = sizingKneeIdealGlobalPosition.Copy()
					debugPositions[debugTargetSizing][debugTypeIdeal][pmx.ANKLE.StringFromDirection(direction)][index] = sizingAnkleIdealGlobalPosition.Copy()
				}

				// IK解決
				sizingLegDeltas, _ := deform.DeformIks(sizingSet.SizingConfigModel, sizingProcessMotion,
					sizingAllDeltas[index], float32(iFrame),
					[]*pmx.Bone{ankleIkBones[d], kneeIkBones[d]},
					[]*pmx.Bone{sizingSet.SizingAnkleBone(direction), sizingSet.SizingKneeBone(direction)},
					[]*mmath.MVec3{sizingAnkleIdealGlobalPosition, sizingKneeIdealGlobalPosition},
					leg_direction_bone_names[d], 1*moveScale.X, false, false)

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

					legDelta := sizingLegDeltas.Bones.GetByName(pmx.LEG.StringFromDirection(direction))
					legRotations[d][index] = legDelta.FilledFrameRotation().Copy()

					ankleDelta := sizingLegDeltas.Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))
					ankleRotations[d][index] = ankleDelta.FilledFrameRotation().Copy()
				}

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
						sizingAllDeltas[index].Bones.Delete(sizingDelta.Bone.Index())
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
		return nil, nil, nil, nil, err
	}

	if mlog.IsDebug() {
		outputDebugData(allFrames, debugBoneNames, verboseMotionKey, sizingSet.OutputMotionPath, sizingSet.SizingConfigModel, debugPositions, debugRotations)
	}

	return legIkPositions, legIkRotations, legRotations, ankleRotations, nil
}

// updateLegIkAndFk は、計算済みの足IK 補正位置と回転値をモーションデータに反映させます。
func (su *SizingLegUsecase) updateLegIkAndFk(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	legIkPositions [][]*mmath.MVec3,
	legIkRotations, legRotations, kneeRotations, ankleRotations, toeExRotations [][]*mmath.MQuaternion,
	incrementCompletedCount func(),
) {
	var emptyPositions [][]*mmath.MVec3

	if legIkPositions == nil {
		legIkPositions = make([][]*mmath.MVec3, 2)
		legIkPositions[0] = make([]*mmath.MVec3, len(allFrames))
		legIkPositions[1] = make([]*mmath.MVec3, len(allFrames))

		for f, iFrame := range allFrames {
			frame := float32(iFrame)
			for d, direction := range directions {
				legIkPositions[d][f] = sizingProcessMotion.BoneFrames.Get(pmx.LEG_IK.StringFromDirection(direction)).Get(frame).Position.Copy()
			}
		}
	}

	for i, iFrame := range allFrames {
		frame := float32(iFrame)

		for _, v := range [][]any{
			{legIkPositions, legIkRotations, pmx.LEG_IK},
			{emptyPositions, legRotations, pmx.LEG},
			{emptyPositions, kneeRotations, pmx.KNEE},
			{emptyPositions, ankleRotations, pmx.ANKLE},
			{emptyPositions, toeExRotations, pmx.TOE_EX},
		} {
			positions := v[0].([][]*mmath.MVec3)
			rotations := v[1].([][]*mmath.MQuaternion)

			if positions == nil && rotations == nil {
				continue
			}

			for d, direction := range directions {
				boneName := v[2].(pmx.StandardBoneName).StringFromDirection(direction)
				if (positions == nil || positions[d][i] == nil) && rotations[d][i] == nil {
					continue
				}

				bf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
				if positions != nil {
					bf.Position = positions[d][i]
				}
				if rotations != nil {
					bf.Rotation = rotations[d][i]
				}
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
	originalAllDeltas, sizingAllDeltas, originalMorphAllDeltas, sizingMorphAllDeltas []*delta.VmdDeltas, legIkPositions [][]*mmath.MVec3,
	sizingProcessMotion *vmd.VmdMotion, incrementCompletedCount func(), debugMotionKey string,
) (
	rootPositions, centerPositions, groovePositions []*mmath.MVec3, isActiveGroove bool,
	err error,
) {
	rootPositions = make([]*mmath.MVec3, len(allFrames))
	centerPositions = make([]*mmath.MVec3, len(allFrames))
	groovePositions = make([]*mmath.MVec3, len(allFrames))

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
		func(index, iFrame int) error {
			if sizingSet.IsTerminate {
				return merr.NewTerminateError("manual terminate")
			}
			frame := float32(iFrame)

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

			// 元との差分
			{
				originalCenterPosition := sizingProcessMotion.BoneFrames.Get(pmx.CENTER.String()).Get(frame).Position
				centerDiff := centerPositions[index].Subed(originalCenterPosition)

				grooveDiff := mmath.MVec3Zero
				if isActiveGroove {
					originalGroovePosition := sizingProcessMotion.BoneFrames.Get(pmx.GROOVE.String()).Get(frame).Position
					grooveDiff = groovePositions[index].Subed(originalGroovePosition)
				}

				legIkPositions[0][index].Add(centerDiff).Add(grooveDiff)
				legIkPositions[1][index].Add(centerDiff).Add(grooveDiff)
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
					sizingAllDeltas[index].Bones.Delete(sizingDelta.Bone.Index())
				}
			}

			incrementCompletedCount()

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正05", sizingSet.Index, iterIndex, allCount)
		})
	if err != nil {
		return nil, nil, nil, isActiveGroove, err
	}

	if mlog.IsDebug() {
		outputDebugData(allFrames, debugBoneNames, debugMotionKey, sizingSet.OutputMotionPath, sizingSet.SizingConfigModel, debugPositions, debugRotations)
	}

	return rootPositions, centerPositions, groovePositions, isActiveGroove, nil
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

func (su *SizingLegUsecase) createFullLegIkBone(sizingSet *domain.SizingSet, direction pmx.BoneDirection) (
	ikBone *pmx.Bone,
) {
	legIkBone, _ := sizingSet.SizingConfigModel.Bones.GetLegIk(direction)
	tailBone, _ := sizingSet.SizingConfigModel.Bones.GetAnkle(direction)
	ikBone = pmx.NewBoneByName(fmt.Sprintf("%s%sIk", pmx.MLIB_PREFIX, tailBone.Name()))

	ikBone.Position = tailBone.Position.Copy()
	ikBone.Ik = pmx.NewIk()
	ikBone.Ik.BoneIndex = tailBone.Index()
	ikBone.Ik.LoopCount = 1000
	ikBone.Ik.UnitRotation = &mmath.MVec3{X: 0.01, Y: 0.0, Z: 0.0}
	ikBone.Ik.Links = make([]*pmx.IkLink, 0)

	for _, boneName := range []string{
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

func (su *SizingLegUsecase) createAnkleIkBone(sizingSet *domain.SizingSet, direction pmx.BoneDirection) (
	ikBone *pmx.Bone,
) {
	tailBone := sizingSet.SizingToeTailBone(direction)
	ankleBone := sizingSet.SizingAnkleBone(direction)
	ikBone = pmx.NewBoneByName(fmt.Sprintf("%s%sIk", pmx.MLIB_PREFIX, tailBone.Name()))

	ikBone.Position = tailBone.Position.Copy()
	ikBone.Ik = pmx.NewIk()
	ikBone.Ik.BoneIndex = tailBone.Index()
	ikBone.Ik.LoopCount = 10
	ikBone.Ik.UnitRotation = &mmath.MVec3{X: 1, Y: 0.0, Z: 0.0}
	ikBone.Ik.Links = make([]*pmx.IkLink, 0)

	for _, boneName := range []string{
		pmx.ANKLE.StringFromDirection(direction),
	} {
		bone, _ := sizingSet.SizingConfigModel.Bones.GetByName(boneName)
		if bone == nil {
			continue
		}

		link := pmx.NewIkLink()
		link.BoneIndex = bone.Index()
		link.AngleLimit = true

		if bone.Index() == ankleBone.Index() {
			// 足首ボーンだけ動かす
			link.AngleLimit = false
		}

		ikBone.Ik.Links = append(ikBone.Ik.Links, link)

		if bone.Index() == ankleBone.Index() {
			// 足首までいったら終了
			break
		}
	}

	return ikBone
}

// calculateAdjustedLegIk は、足IK 補正の計算を並列処理で行い、各フレームごとの位置・回転補正値を算出します。
func (su *SizingLegUsecase) calculateAdjustedLegIk2(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, moveScale *mmath.MVec3,
	originalAllDeltas, sizingLegIkOffAllDeltas, sizingLegIkOnAllDeltas,
	originalMorphAllDeltas, sizingMorphAllDeltas []*delta.VmdDeltas,
	sizingProcessMotion *vmd.VmdMotion, incrementCompletedCount func(), verboseMotionKey string,
) (legIkPositions [][]*mmath.MVec3,
	legIkRotations, legRotations, kneeRotations, ankleRotations, toeExRotations [][]*mmath.MQuaternion,
	err error) {
	legIkPositions = make([][]*mmath.MVec3, 2)
	legIkRotations = make([][]*mmath.MQuaternion, 2)
	legRotations = make([][]*mmath.MQuaternion, 2)
	kneeRotations = make([][]*mmath.MQuaternion, 2)
	ankleRotations = make([][]*mmath.MQuaternion, 2)
	toeExRotations = make([][]*mmath.MQuaternion, 2)

	for d := range directions {
		legIkPositions[d] = make([]*mmath.MVec3, len(allFrames))
		legIkRotations[d] = make([]*mmath.MQuaternion, len(allFrames))
		legRotations[d] = make([]*mmath.MQuaternion, len(allFrames))
		kneeRotations[d] = make([]*mmath.MQuaternion, len(allFrames))
		ankleRotations[d] = make([]*mmath.MQuaternion, len(allFrames))
		toeExRotations[d] = make([]*mmath.MQuaternion, len(allFrames))
	}

	// isActiveToeEx := false
	// for _, direction := range directions {
	// 	sizingProcessMotion.BoneFrames.Get(pmx.TOE_EX.StringFromDirection(direction)).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
	// 		if !bf.FilledRotation().IsIdent() {
	// 			isActiveToeEx = true
	// 			return false
	// 		}
	// 		return true
	// 	})
	// }

	legIkBones := make([]*pmx.Bone, 2)
	legIkBones[0] = su.createFullLegIkBone(sizingSet, pmx.BONE_DIRECTION_LEFT)
	legIkBones[1] = su.createFullLegIkBone(sizingSet, pmx.BONE_DIRECTION_RIGHT)

	ankleIkBones := make([]*pmx.Bone, 2)
	ankleIkBones[0] = su.createAnkleIkBone(sizingSet, pmx.BONE_DIRECTION_LEFT)
	ankleIkBones[1] = su.createAnkleIkBone(sizingSet, pmx.BONE_DIRECTION_RIGHT)

	toeIkTargetBones := make([]*pmx.Bone, 2)
	toeIkTargetBones[0], _ = sizingSet.SizingConfigModel.Bones.Get(
		sizingSet.SizingToeIkBone(pmx.BONE_DIRECTION_LEFT).Ik.BoneIndex)
	toeIkTargetBones[1], _ = sizingSet.SizingConfigModel.Bones.Get(
		sizingSet.SizingToeIkBone(pmx.BONE_DIRECTION_RIGHT).Ik.BoneIndex)

	debugBoneNames := []pmx.StandardBoneName{
		pmx.LEG, pmx.ANKLE, pmx.TOE_T, pmx.HEEL, pmx.LEG_IK, pmx.TOE_EX,
	}
	debugPositions, debugRotations := newDebugData(allFrames, debugBoneNames)
	debugVectorPositions := make([][][]*mmath.MVec3, 5)
	for i := range 5 {
		debugVectorPositions[i] = make([][]*mmath.MVec3, 2)
		for d := range directions {
			debugVectorPositions[i][d] = make([]*mmath.MVec3, len(allFrames))
		}
	}

	err = miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, iFrame int) error {
			if sizingSet.IsTerminate {
				return merr.NewTerminateError("manual terminate")
			}

			originalLeftLegDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG.Left())
			originalRightLegDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG.Right())
			originalLeftAnkleDelta := originalAllDeltas[index].Bones.GetByName(pmx.ANKLE.Left())
			originalRightAnkleDelta := originalAllDeltas[index].Bones.GetByName(pmx.ANKLE.Right())
			originalLegDistance := originalLeftLegDelta.FilledGlobalPosition().Distance(
				originalRightLegDelta.FilledGlobalPosition())
			originalAnkleDistance := originalLeftAnkleDelta.FilledGlobalPosition().Distance(
				originalRightAnkleDelta.FilledGlobalPosition())

			for d, direction := range directions {
				// 足
				originalMorphLegDelta := originalMorphAllDeltas[index].Bones.GetByName(
					pmx.LEG.StringFromDirection(direction))
				sizingMorphLegDelta := sizingMorphAllDeltas[index].Bones.GetByName(
					pmx.LEG.StringFromDirection(direction))

				originalMorphLegIkDelta := originalMorphAllDeltas[index].Bones.GetByName(
					pmx.LEG_IK.StringFromDirection(direction))
				originalMorphAnkleDelta := originalMorphAllDeltas[index].Bones.GetByName(
					pmx.ANKLE.StringFromDirection(direction))
				sizingMorphAnkleDelta := sizingMorphAllDeltas[index].Bones.GetByName(
					pmx.ANKLE.StringFromDirection(direction))

				// 足首地面の長さ差
				originalAnkleLength := originalMorphAnkleDelta.FilledGlobalPosition().Y
				sizingAnkleLength := sizingMorphAnkleDelta.FilledGlobalPosition().Y
				ankleScale := sizingAnkleLength / originalAnkleLength

				// 足から足首までの長さ差
				originalLegLength := originalMorphLegDelta.FilledGlobalPosition().Distance(
					originalMorphAnkleDelta.FilledGlobalPosition())
				sizingLegLength := sizingMorphLegDelta.FilledGlobalPosition().Distance(
					sizingMorphAnkleDelta.FilledGlobalPosition())
				legScale := sizingLegLength / originalLegLength

				originalMorphHeelDelta := originalMorphAllDeltas[index].Bones.GetByName(
					pmx.HEEL.StringFromDirection(direction))
				originalMorphToeTailDelta := originalMorphAllDeltas[index].Bones.GetByName(
					pmx.TOE_T.StringFromDirection(direction))
				sizingMorphHeelDelta := sizingMorphAllDeltas[index].Bones.GetByName(
					pmx.HEEL.StringFromDirection(direction))
				sizingMorphToeTailDelta := sizingMorphAllDeltas[index].Bones.GetByName(
					pmx.TOE_T.StringFromDirection(direction))

				originalHeelLength := originalMorphHeelDelta.FilledGlobalPosition().Distance(
					originalMorphAnkleDelta.FilledGlobalPosition())
				sizingHeelLength := sizingMorphHeelDelta.FilledGlobalPosition().Distance(
					sizingMorphAnkleDelta.FilledGlobalPosition())
				heelScale := sizingHeelLength / originalHeelLength

				originalSoleLength := originalMorphHeelDelta.FilledGlobalPosition().Distance(
					originalMorphToeTailDelta.FilledGlobalPosition())
				sizingSoleLength := sizingMorphHeelDelta.FilledGlobalPosition().Distance(
					sizingMorphToeTailDelta.FilledGlobalPosition())
				soleScale := sizingSoleLength / originalSoleLength

				// -----------------------------

				// 足IKがターゲットより伸びている場合の対応
				originalLegDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG.StringFromDirection(direction))
				originalLegRootDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG_ROOT.StringFromDirection(direction))
				originalLegIkDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG_IK.StringFromDirection(direction))
				originalAnkleDelta := originalAllDeltas[index].Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))
				originalLegIkDiff := originalLegIkDelta.FilledGlobalPosition().Subed(originalAnkleDelta.FilledGlobalPosition())
				// 先モデルにおける足首から見た足IKの差分を求める
				sizingLegIkDiff := originalLegIkDiff.Muled(moveScale)

				originalHeelDelta := originalAllDeltas[index].Bones.GetByName(pmx.HEEL_D.StringFromDirection(direction))
				originalToeTailDelta := originalAllDeltas[index].Bones.GetByName(pmx.TOE_T_D.StringFromDirection(direction))
				originalToePDelta := originalAllDeltas[index].Bones.GetByName(pmx.TOE_P_D.StringFromDirection(direction))

				// 元の足首からかかとの傾き
				originalHeelVector := originalHeelDelta.FilledGlobalPosition().Subed(originalAnkleDelta.FilledGlobalPosition())
				// 元のかかとからつま先の傾き
				originalToeTailVector := originalToeTailDelta.FilledGlobalPosition().Subed(originalHeelDelta.FilledGlobalPosition())

				sizingIdealHeelVector := originalHeelVector.MuledScalar(heelScale)
				sizingIdealToeTailVector := originalToeTailVector.MuledScalar(soleScale)
				originalToePVector := originalToePDelta.FilledGlobalPosition().Subed(originalToeTailDelta.FilledGlobalPosition())

				// --------------------------------

				// [元モデル]足根元から見た足首の相対位置
				originalLegAnkleLocalPosition := originalLegRootDelta.FilledGlobalMatrix().Inverted().MulVec3(
					originalAnkleDelta.FilledGlobalPosition()).Normalized()

				// --------------------------------
				// FKベースで足IKの位置を計算するパターン

				legDelta := sizingLegIkOffAllDeltas[index].Bones.GetByName(pmx.LEG.StringFromDirection(direction))
				legRootDelta := sizingLegIkOffAllDeltas[index].Bones.GetByName(pmx.LEG_ROOT.StringFromDirection(direction))
				ankleDelta := sizingLegIkOffAllDeltas[index].Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))
				sizingAnkleIdealGlobalPositionByAnkle := ankleDelta.FilledGlobalPosition()

				// 一旦この時点での理想位置を求める
				sizingHeelPreIdealoGlobalPositionByAnkle := sizingAnkleIdealGlobalPositionByAnkle.Added(sizingIdealHeelVector)
				sizingToePreIdealGlobalPositionByAnkle := sizingHeelPreIdealoGlobalPositionByAnkle.Added(sizingIdealToeTailVector)

				sizingAnkleIdealGlobalPositionByAnkle.Y += su.calculateAnkleYDiff(
					originalMorphAllDeltas[index], originalAllDeltas[index], direction, ankleScale,
					sizingHeelPreIdealoGlobalPositionByAnkle.Y, sizingToePreIdealGlobalPositionByAnkle.Y)
				if mmath.NearEquals(originalLegIkDelta.FilledGlobalPosition().Y, originalMorphLegIkDelta.FilledGlobalPosition().Y, 1e-2) {
					// 足首が動いていない場合、足IKを動かさない
					sizingAnkleIdealGlobalPositionByAnkle.Y = sizingMorphAnkleDelta.FilledGlobalPosition().Y
				}

				// [先モデル][FKベース]足根元から見た足首の相対位置
				sizingByAnkleLegAnkleLocalPosition := legRootDelta.FilledGlobalMatrix().Inverted().MulVec3(
					sizingAnkleIdealGlobalPositionByAnkle)

				// --------------------------------
				// 足からみた足首のローカル位置ベースで足IKの位置を計算するパターン

				// 足から見た足首のローカル位置
				ankleLocalPosition := originalLegDelta.FilledGlobalMatrix().Inverted().MulVec3(
					originalAnkleDelta.FilledGlobalPosition())
				scaledAnkleLocalPosition := ankleLocalPosition.MuledScalar(legScale)

				sizingAnkleIdealGlobalPositionByLegIk := legDelta.FilledGlobalMatrix().MulVec3(scaledAnkleLocalPosition)

				// 一旦この時点での理想位置を求める
				sizingHeelPreIdealoGlobalPositionByLegIk := sizingAnkleIdealGlobalPositionByLegIk.Added(sizingIdealHeelVector)
				sizingToePreIdealGlobalPositionByLegIk := sizingHeelPreIdealoGlobalPositionByLegIk.Added(sizingIdealToeTailVector)

				sizingAnkleIdealGlobalPositionByLegIk.Y += su.calculateAnkleYDiff(
					originalMorphAllDeltas[index], originalAllDeltas[index], direction, ankleScale,
					sizingHeelPreIdealoGlobalPositionByLegIk.Y, sizingToePreIdealGlobalPositionByLegIk.Y)
				if mmath.NearEquals(originalLegIkDelta.FilledGlobalPosition().Y, originalMorphLegIkDelta.FilledGlobalPosition().Y, 1e-2) {
					// 足首が動いていない場合、足IKを動かさない
					sizingAnkleIdealGlobalPositionByLegIk.Y = sizingMorphAnkleDelta.FilledGlobalPosition().Y
				}

				// [先モデル][IKベース]足根元から見た足首の相対位置
				sizingByLegIkLegAnkleLocalPosition := legRootDelta.FilledGlobalMatrix().Inverted().MulVec3(
					sizingAnkleIdealGlobalPositionByLegIk)

				// 元モデルの足首の距離が近い場合には、足IKベースのXに合わせる
				ankleIdealLocalX := mmath.Lerp(sizingByLegIkLegAnkleLocalPosition.X, sizingByAnkleLegAnkleLocalPosition.X, originalAnkleDistance-originalLegDistance)

				// ローカルXのみ足IKベースの足首位置を加味する（足から見た足首が交差しないように）
				sizingAnkleIdealLocalPosition := &mmath.MVec3{
					X: ankleIdealLocalX,
					Y: sizingByAnkleLegAnkleLocalPosition.Y,
					Z: sizingByAnkleLegAnkleLocalPosition.Z,
				}

				if mlog.IsDebug() {
					debugVectorPositions[0][d][index] = originalLegAnkleLocalPosition.Copy()
					debugVectorPositions[1][d][index] = sizingByAnkleLegAnkleLocalPosition.Copy()
					debugVectorPositions[2][d][index] = sizingByLegIkLegAnkleLocalPosition.Copy()
					debugVectorPositions[3][d][index] = sizingAnkleIdealLocalPosition.Copy()
					debugVectorPositions[4][d][index] = &mmath.MVec3{
						X: originalAnkleDistance,
						Y: originalLegDistance,
						Z: originalAnkleDistance - originalLegDistance,
					}
				}

				sizingAnkleIdealGlobalPosition := legRootDelta.FilledGlobalMatrix().MulVec3(sizingAnkleIdealLocalPosition)

				// つま先IK用に足IKを伸ばさなかった場合の足首位置を保持
				sizingAnkleFitIdealGlobalPosition := sizingAnkleIdealGlobalPosition.Copy()
				// 足IKのターゲットとしては、足IKを伸ばした場合を加味する
				sizingAnkleIdealGlobalPosition.Add(sizingLegIkDiff)

				// 最終的な理想位置を求める
				sizingHeelIdealoGlobalPosition := sizingAnkleFitIdealGlobalPosition.Added(sizingIdealHeelVector)
				sizingToeIdealGlobalPosition := sizingHeelIdealoGlobalPosition.Added(sizingIdealToeTailVector)
				sizingToePIdealGlobalPosition := sizingToeIdealGlobalPosition.Added(originalToePVector)

				if mlog.IsDebug() {
					recordDebugData(index, debugBoneNames, originalAllDeltas[index],
						debugTargetOriginal, debugTypeInitial, debugPositions, debugRotations)
					recordDebugData(index, debugBoneNames, sizingLegIkOffAllDeltas[index],
						debugTargetSizing, debugTypeInitial, debugPositions, debugRotations)

					debugPositions[debugTargetSizing][debugTypeIdeal][pmx.ANKLE.StringFromDirection(direction)][index] = sizingAnkleIdealGlobalPosition.Copy()
					debugPositions[debugTargetSizing][debugTypeIdeal][pmx.HEEL.StringFromDirection(direction)][index] = sizingHeelIdealoGlobalPosition.Copy()
					debugPositions[debugTargetSizing][debugTypeIdeal][pmx.TOE_T.StringFromDirection(direction)][index] = sizingToeIdealGlobalPosition.Copy()
				}

				// IK解決
				sizingLegDeltas, _ := deform.DeformIks(sizingSet.SizingConfigModel, sizingProcessMotion,
					sizingLegIkOffAllDeltas[index], float32(iFrame),
					[]*pmx.Bone{legIkBones[d], ankleIkBones[d]},
					[]*pmx.Bone{sizingSet.SizingAnkleBone(direction), sizingSet.SizingToeTailBone(direction)},
					[]*mmath.MVec3{sizingAnkleIdealGlobalPosition, sizingToeIdealGlobalPosition},
					leg_direction_bone_names[d], 1*moveScale.X, false, false)

				// 全親からみた足IKデフォルト位置からみた現在の足首のローカル位置
				sizingLegIkMorphDelta := sizingMorphAllDeltas[index].Bones.Get(sizingSet.SizingLegIkBone(direction).Index())
				sizingRootMorphDelta := sizingMorphAllDeltas[index].Bones.GetByName(sizingSet.SizingRootBone().Name())
				sizingLegIkMorphLocalPosition :=
					sizingRootMorphDelta.FilledGlobalMatrix().Inverted().MulVec3(
						sizingLegIkMorphDelta.FilledGlobalPosition())

				sizingAnkleDelta := sizingLegDeltas.Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))
				sizingRootDelta := sizingLegDeltas.Bones.GetByName(sizingSet.SizingRootBone().Name())
				legIkPositions[d][index] = sizingRootDelta.FilledGlobalMatrix().Inverted().MulVec3(sizingAnkleDelta.FilledGlobalPosition()).Subed(sizingLegIkMorphLocalPosition).Added(sizingLegIkDiff)

				// 元の足IK=Yのグローバル位置が0の場合、足IKのYを0にする
				if mmath.NearEquals(originalLegIkDelta.FilledGlobalPosition().Y,
					sizingSet.OriginalLegIkBone(direction).Position.Y, 1e-2) {
					legIkPositions[d][index].Y = 0.0
				}

				legRotations[d][index] = sizingLegDeltas.Bones.GetByName(pmx.LEG.StringFromDirection(direction)).FilledFrameRotation().Copy()
				kneeRotations[d][index] = sizingLegDeltas.Bones.GetByName(pmx.KNEE.StringFromDirection(direction)).FilledFrameRotation().Copy()
				ankleRotations[d][index] = sizingLegDeltas.Bones.GetByName(pmx.ANKLE.StringFromDirection(direction)).FilledFrameRotation().Copy()

				// -----------------------------
				// 足IKの回転量計算

				// かかとからつま先に向けた初期ベクトル
				sizingHeelInitialDelta := sizingMorphAllDeltas[index].Bones.GetByName(
					pmx.HEEL.StringFromDirection(direction))
				sizingToeTInitialDelta := sizingMorphAllDeltas[index].Bones.GetByName(
					pmx.TOE_T.StringFromDirection(direction))
				sizingToeTInitialLocalPosition := sizingToeTInitialDelta.FilledGlobalPosition().Subed(
					sizingHeelInitialDelta.FilledGlobalPosition())

				// つま先からつま先親に向けた初期ベクトル
				sizingToePInitialDelta := sizingMorphAllDeltas[index].Bones.GetByName(
					pmx.TOE_P.StringFromDirection(direction))
				sizingToePInitialLocalPosition := sizingToePInitialDelta.FilledGlobalPosition().Subed(
					sizingToeTInitialDelta.FilledGlobalPosition())

				// かかとからつま先に向けた理想ベクトル
				sizingToeTIdealLocalPosition := sizingToeIdealGlobalPosition.Subed(sizingHeelIdealoGlobalPosition)

				// つま先からつま先親に向けた理想ベクトル
				sizingToePIdealLocalPosition := sizingToePIdealGlobalPosition.Subed(sizingToeIdealGlobalPosition)

				// 初期回転
				toeInitialSlope := mmath.NewMQuaternionFromDirection(
					sizingToeTInitialLocalPosition, sizingToePInitialLocalPosition)
				// 理想回転
				toeIdealSlope := mmath.NewMQuaternionFromDirection(
					sizingToeTIdealLocalPosition, sizingToePIdealLocalPosition)

				// 回転差分
				legIkRotations[d][index] = toeIdealSlope.Muled(toeInitialSlope.Inverted())

				// // -----------------------------
				// // 足先EXの回転量再計算

				// if isActiveToeEx {
				// 	// 足先EXの親から足先EXの初期位置を伸ばす
				// 	sizingToeExParentDelta := sizingLegIkOnAllDeltas[index].Bones.GetByName(
				// 		sizingSet.SizingToeExBone(direction).ParentBone.Name())
				// 	sizingToeTailDInitialPosition := sizingToeExParentDelta.FilledGlobalMatrix().MulVec3(
				// 		sizingSet.SizingToeTailDBone(direction).Position.Subed(
				// 			sizingSet.SizingToeExBone(direction).ParentBone.Position))

				// 	sizingToeExDelta := sizingLegIkOnAllDeltas[index].Bones.GetByName(
				// 		pmx.TOE_EX.StringFromDirection(direction))

				// 	sizingToeTailInitialRelativePosition := sizingToeTailDInitialPosition.Subed(
				// 		sizingToeExDelta.FilledGlobalPosition()).Normalized()
				// 	sizingToeTailIdealRelativePosition := sizingToeIdealGlobalPosition.Subed(
				// 		sizingToeExDelta.FilledGlobalPosition()).Normalized()

				// 	// つま先EXの理想角度
				// 	toeExRotations[d][index] = mmath.NewMQuaternionRotate(
				// 		sizingToeTailInitialRelativePosition, sizingToeTailIdealRelativePosition)

				// 	if mlog.IsDebug() {
				// 		debugPositions[debugTargetOriginal][debugTypeInitial][pmx.TOE_EX.StringFromDirection(direction)][index] = sizingToeTailDInitialPosition.Copy()
				// 		debugPositions[debugTargetSizing][debugTypeInitial][pmx.TOE_EX.StringFromDirection(direction)][index] = sizingToeIdealGlobalPosition.Copy()

				// 		debugPositions[debugTargetSizing][debugTypeResult][pmx.TOE_EX.StringFromDirection(direction)][index] = sizingToeExDelta.FilledGlobalPosition().Copy()
				// 		debugRotations[debugTargetSizing][debugTypeResult][pmx.TOE_EX.StringFromDirection(direction)][index] = toeExRotations[d][index].Copy()
				// 	}
				// }

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
						sizingLegIkOnAllDeltas[index].Bones.Delete(sizingDelta.Bone.Index())
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

		motion := vmd.NewVmdMotion("")
		for d, direction := range directions {
			for _, iFrame := range allFrames {
				frame := float32(iFrame)
				for i, prefix := range []string{"元", "元距", "先FK", "先IK", "先理"} {
					bf := vmd.NewBoneFrame(frame)
					bf.Position = debugVectorPositions[i][d][iFrame]
					if bf.Position != nil || bf.Rotation != nil {
						motion.InsertBoneFrame(fmt.Sprintf("%s%s足V", prefix, direction), bf)
					}
				}
			}
		}
		outputVerboseMotion(fmt.Sprintf("%sA", verboseMotionKey), sizingSet.OutputMotionPath, motion)
	}

	return legIkPositions, legIkRotations, legRotations, kneeRotations, ankleRotations, toeExRotations, nil
}

func (su *SizingLegUsecase) calculateAnkleYDiff(
	originalMorphDelta, originalDelta *delta.VmdDeltas,
	direction pmx.BoneDirection, ankleScale, actualSizingHeelY, actualSizingToeTailY float64,
) float64 {
	originalMorphAnkleDelta := originalMorphDelta.Bones.GetByName(pmx.ANKLE_D.StringFromDirection(direction))
	originalToeTailDelta := originalDelta.Bones.GetByName(pmx.TOE_T_D.StringFromDirection(direction))
	originalHeelDelta := originalDelta.Bones.GetByName(pmx.HEEL_D.StringFromDirection(direction))

	originalMorphAnkleY := originalMorphAnkleDelta.FilledGlobalPosition().Y
	originalToeTailDY := originalToeTailDelta.FilledGlobalPosition().Y
	originalHeelDY := originalHeelDelta.FilledGlobalPosition().Y

	// つま先の補正値 ------------------
	// つま先のY座標を元モデルのつま先のY座標*スケールに合わせる
	idealSizingToeTailY := originalToeTailDY * ankleScale

	toeDiff := idealSizingToeTailY - actualSizingToeTailY
	lerpToeDiff := mmath.Lerp(toeDiff, 0, originalToeTailDY/originalMorphAnkleY)

	// かかとの補正値 ------------------
	// かかとのY座標を元モデルのかかとのY座標*スケールに合わせる
	idealSizingHeelY := originalHeelDY * ankleScale

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

// // calculateAdjustedLegIk は、足IK 補正の計算を並列処理で行い、各フレームごとの位置・回転補正値を算出します。
// func (su *SizingLegUsecase) calculateAdjustedLegIk3(
// 	sizingSet *domain.SizingSet, allFrames []int, blockSize int, sizingLegIkOnAllDeltas []*delta.VmdDeltas,
// 	incrementCompletedCount func(),
// ) (legIkRotations [][]*mmath.MQuaternion, err error) {
// 	legIkRotations = make([][]*mmath.MQuaternion, 2)

// 	for d := range directions {
// 		legIkRotations[d] = make([]*mmath.MQuaternion, len(allFrames))
// 	}

// 	err = miter.IterParallelByList(allFrames, blockSize, log_block_size,
// 		func(index, iFrame int) error {
// 			if sizingSet.IsTerminate {
// 				return merr.NewTerminateError("manual terminate")
// 			}

// 			for d, direction := range directions {
// 				sizingAnkleDelta := sizingLegIkOnAllDeltas[index].Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))
// 				legIkRotations[d][index] = sizingAnkleDelta.FilledGlobalMatrix().Quaternion()
// 			}

// 			incrementCompletedCount()

// 			return nil
// 		},
// 		func(iterIndex, allCount int) {
// 			processLog("足補正07", sizingSet.Index, iterIndex, allCount)
// 		})
// 	if err != nil {
// 		return nil, err
// 	}

// 	return legIkRotations, nil
// }

// // updateLegIkAndFk2 は、計算済みの足IK 補正位置と回転値をモーションデータに反映させます。
// func (su *SizingLegUsecase) updateLegIkAndFk2(
// 	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
// 	legIkPositions [][]*mmath.MVec3,
// 	legIkRotations, legRotations, kneeRotations, ankleRotations [][]*mmath.MQuaternion,
// 	incrementCompletedCount func(),
// ) {
// 	emptyPositions := make([][]*mmath.MVec3, 2)
// 	emptyPositions[0] = make([]*mmath.MVec3, len(allFrames))
// 	emptyPositions[1] = make([]*mmath.MVec3, len(allFrames))

// 	for i, iFrame := range allFrames {
// 		frame := float32(iFrame)

// 		for _, v := range [][]any{
// 			{legIkPositions, legIkRotations, pmx.LEG_IK},
// 			{emptyPositions, legRotations, pmx.LEG},
// 			{emptyPositions, kneeRotations, pmx.KNEE},
// 			{emptyPositions, ankleRotations, pmx.ANKLE},
// 		} {
// 			positions := v[0].([][]*mmath.MVec3)
// 			rotations := v[1].([][]*mmath.MQuaternion)
// 			for d, direction := range directions {
// 				boneName := v[2].(pmx.StandardBoneName).StringFromDirection(direction)

// 				bf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
// 				bf.Position = positions[d][i]
// 				bf.Rotation = rotations[d][i]
// 				sizingProcessMotion.InsertBoneFrame(boneName, bf)
// 			}
// 		}

// 		if i > 0 && i%1000 == 0 {
// 			processLog("足補正08", sizingSet.Index, i, len(allFrames))
// 		}

//			incrementCompletedCount()
//		}
//	}

type boneCheck struct {
	checkBoneNames  []pmx.StandardBoneName
	thresholds      []float64
	updateBoneNames []pmx.StandardBoneName
}

func (su *SizingLegUsecase) updateOutputMotion(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, isActiveGroove bool,
	sizingProcessMotion *vmd.VmdMotion, legScale float64, verboseMotionKey string, incrementCompletedCount func(),
) error {
	// 足補正処理の結果をサイジング先モーションに反映
	sizingModel := sizingSet.SizingConfigModel
	outputMotion := sizingSet.OutputMotion

	targetBoneNames := []string{
		pmx.ROOT.String(), pmx.CENTER.String(), pmx.GROOVE.String(), pmx.LOWER.String(),
		pmx.LEG_IK.Left(), pmx.LEG.Left(), pmx.KNEE.Left(), pmx.ANKLE.Left(),
		pmx.LEG_IK.Right(), pmx.LEG.Right(), pmx.KNEE.Right(), pmx.ANKLE.Right(),
	}

	// activeFrames := getFrames(outputMotion, targetBoneNames)
	intervalFrames := mmath.IntRangesByStep(allFrames[0], allFrames[len(allFrames)-1], 4)

	// 足IK親・つま先IKの除去
	outputMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.LEG_IK_PARENT.Left()))
	outputMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.LEG_IK_PARENT.Right()))
	outputMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.TOE_IK.Left()))
	outputMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.TOE_IK.Right()))

	processAllDeltas, err := computeVmdDeltas(allFrames, blockSize,
		sizingModel, sizingProcessMotion, sizingSet, true, all_lower_leg_bone_names, "足補正01", incrementCompletedCount)
	if err != nil {
		return err
	}

	debugBoneNames := []pmx.StandardBoneName{
		pmx.LOWER, pmx.LEG, pmx.KNEE, pmx.ANKLE, pmx.LEG_IK,
		pmx.TOE_EX, pmx.TOE_T_D, pmx.HEEL_D, pmx.TOE_P_D, pmx.TOE_C_D,
	}
	debugPositions, debugRotations := newDebugData(allFrames, debugBoneNames)

	// 足系はあるボーンだけ上書きする(足先EXはあるキーフレだけ更新する)
	for _, boneName := range append(targetBoneNames, pmx.TOE_EX.Left(), pmx.TOE_EX.Right()) {
		if !outputMotion.BoneFrames.Contains(boneName) {
			continue
		}
		outputMotion.BoneFrames.Get(boneName).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
			processBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
			if processBf == nil {
				return true
			}
			bf.Position = processBf.FilledPosition().Copy()
			bf.Rotation = processBf.FilledRotation().Copy()
			outputMotion.InsertBoneFrame(boneName, bf)
			return true
		})
	}

	// 中間キーフレのズレをチェック
	lowerThreshold := 0.2 * legScale
	legThreshold := 0.2 * legScale
	kneeThreshold := 0.2 * legScale
	ankleThreshold := 0.2 * legScale
	heelThreshold := 0.2 * legScale
	toePDThreshold := 0.2 * legScale
	toeCDThreshold := 0.2 * legScale

	// 体幹系
	for _, v := range []boneCheck{
		{
			checkBoneNames:  []pmx.StandardBoneName{pmx.StandardBoneName(pmx.LOWER.String())},
			thresholds:      []float64{lowerThreshold},
			updateBoneNames: []pmx.StandardBoneName{pmx.CENTER, pmx.GROOVE},
		},
		{
			checkBoneNames:  []pmx.StandardBoneName{pmx.StandardBoneName(pmx.LEG.Left()), pmx.StandardBoneName(pmx.LEG.Right())},
			thresholds:      []float64{legThreshold, legThreshold},
			updateBoneNames: []pmx.StandardBoneName{pmx.LOWER},
		},
	} {
		for tIndex, targetFrames := range [][]int{intervalFrames, allFrames, allFrames} {
			prevLog := 0
			prevFrame := 0
			for fIndex, iFrame := range targetFrames {
				if sizingSet.IsTerminate {
					return merr.NewTerminateError("manual terminate")
				}
				frame := float32(iFrame)

				// 現時点の結果
				resultDeltas, err := computeVmdDeltas([]int{iFrame}, 1,
					sizingModel, outputMotion, sizingSet, true, trunk_lower_bone_names, "", nil)
				if err != nil {
					return err
				}

				// 位置をチェック
				isUpdate := false
				for i, boneName := range v.checkBoneNames {
					resultDelta := resultDeltas[0].Bones.GetByName(boneName.String())
					processDelta := processAllDeltas[iFrame].Bones.GetByName(boneName.String())

					if resultDelta.FilledGlobalPosition().Distance(processDelta.FilledGlobalPosition()) > v.thresholds[i] {
						isUpdate = true
						break
					}
				}

				// 更新が必要な場合
				if isUpdate {
					if mlog.IsDebug() {
						recordDebugData(iFrame, v.checkBoneNames, processAllDeltas[iFrame],
							debugTargetOriginal, debugTypeInitial, debugPositions, debugRotations)
						recordDebugData(iFrame, v.checkBoneNames, resultDeltas[0],
							debugTargetSizing, debugTypeInitial, debugPositions, debugRotations)
					}

					for _, name := range v.updateBoneNames {
						if name == pmx.GROOVE && !isActiveGroove {
							continue
						}

						boneName := name.String()
						processBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
						resultBf := outputMotion.BoneFrames.Get(boneName).Get(frame)
						resultBf.Position = processBf.FilledPosition().Copy()
						resultBf.Rotation = processBf.FilledRotation().Copy()
						outputMotion.InsertBoneFrame(boneName, resultBf)
					}
				}

				if fIndex > 0 && int(iFrame/1000) > prevLog {
					mlog.I(mi18n.T("足補正11", map[string]any{
						"No":             sizingSet.Index + 1,
						"IterIndex":      fmt.Sprintf("%04d", iFrame),
						"AllCount":       fmt.Sprintf("%04d", allFrames[len(allFrames)-1]),
						"UpdateBoneName": v.updateBoneNames[0].String(),
						"FramesIndex":    tIndex + 1}))
					prevLog = int(iFrame / 1000)
				}

				for f := prevFrame + 1; f <= iFrame; f++ {
					incrementCompletedCount()
				}

				prevFrame = iFrame
			}
		}
	}

	// 左右系
	for d, direction := range directions {
		for _, v := range []boneCheck{
			{
				checkBoneNames:  []pmx.StandardBoneName{pmx.ANKLE},
				thresholds:      []float64{ankleThreshold},
				updateBoneNames: []pmx.StandardBoneName{pmx.LEG_IK},
			},
			{
				checkBoneNames:  []pmx.StandardBoneName{pmx.KNEE},
				thresholds:      []float64{kneeThreshold},
				updateBoneNames: []pmx.StandardBoneName{pmx.LEG},
			},
			{
				checkBoneNames:  []pmx.StandardBoneName{pmx.ANKLE},
				thresholds:      []float64{ankleThreshold},
				updateBoneNames: []pmx.StandardBoneName{pmx.KNEE},
			},
			{
				checkBoneNames:  []pmx.StandardBoneName{pmx.TOE_P_D, pmx.TOE_C_D, pmx.HEEL_D},
				thresholds:      []float64{toePDThreshold, toeCDThreshold, heelThreshold},
				updateBoneNames: []pmx.StandardBoneName{pmx.ANKLE, pmx.LEG_IK},
			},
		} {
			for tIndex, targetFrames := range [][]int{intervalFrames, allFrames, allFrames} {
				prevLog := 0
				prevFrame := 0
				for fIndex, iFrame := range targetFrames {
					if sizingSet.IsTerminate {
						return merr.NewTerminateError("manual terminate")
					}
					frame := float32(iFrame)

					// 現時点の結果
					resultDeltas, err := computeVmdDeltas([]int{iFrame}, 1,
						sizingModel, outputMotion, sizingSet, true, leg_direction_bone_names[d], "", nil)
					if err != nil {
						return err
					}

					// 位置をチェック
					isUpdate := false
					for i, boneName := range v.checkBoneNames {
						resultDelta := resultDeltas[0].Bones.GetByName(boneName.StringFromDirection(direction))
						processDelta := processAllDeltas[iFrame].Bones.GetByName(boneName.StringFromDirection(direction))

						if resultDelta.FilledGlobalPosition().Distance(processDelta.FilledGlobalPosition()) > v.thresholds[i] {
							isUpdate = true
							break
						}
					}

					// 更新が必要な場合
					if isUpdate {
						if mlog.IsDebug() {
							recordDebugData(iFrame, v.checkBoneNames, processAllDeltas[iFrame],
								debugTargetOriginal, debugTypeInitial, debugPositions, debugRotations)
							recordDebugData(iFrame, v.checkBoneNames, resultDeltas[0],
								debugTargetSizing, debugTypeInitial, debugPositions, debugRotations)
						}

						for _, name := range v.updateBoneNames {
							if name == pmx.GROOVE && !isActiveGroove {
								continue
							}

							boneName := name.StringFromDirection(direction)
							processBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
							resultBf := outputMotion.BoneFrames.Get(boneName).Get(frame)
							resultBf.Position = processBf.FilledPosition().Copy()
							resultBf.Rotation = processBf.FilledRotation().Copy()
							outputMotion.InsertBoneFrame(boneName, resultBf)
						}
					}

					if fIndex > 0 && int(iFrame/1000) > prevLog {
						mlog.I(mi18n.T("足補正11", map[string]any{
							"No":             sizingSet.Index + 1,
							"IterIndex":      fmt.Sprintf("%04d", iFrame),
							"AllCount":       fmt.Sprintf("%04d", allFrames[len(allFrames)-1]),
							"UpdateBoneName": v.updateBoneNames[0].StringFromDirection(direction),
							"FramesIndex":    tIndex + 1}))
						prevLog = int(iFrame / 1000)
					}

					for f := prevFrame + 1; f <= iFrame; f++ {
						incrementCompletedCount()
					}

					prevFrame = iFrame
				}
			}
		}
	}

	if mlog.IsDebug() {
		outputDebugData(allFrames, debugBoneNames, verboseMotionKey, sizingSet.OutputMotionPath, sizingSet.SizingConfigModel, debugPositions, debugRotations)
	}

	return nil
}

func (su *SizingLegUsecase) updateLegIkOffset(
	sizingSet *domain.SizingSet, originalAllDeltas []*delta.VmdDeltas,
	allFrames []int, legScale float64, verboseMotionKey string,
) {
	// 元モーションのIKが動いていない区間を取得
	fixIkFlags := make([][]bool, len(directions))
	fixIkFlags[0] = make([]bool, len(allFrames))
	fixIkFlags[1] = make([]bool, len(allFrames))

	debugBoneNames := []pmx.StandardBoneName{pmx.ANKLE}
	debugPositions, debugRotations := newDebugData(allFrames, debugBoneNames)

	for i, originalDeltas := range originalAllDeltas {
		for d, direction := range directions {
			if i == 0 {
				fixIkFlags[d][i] = false

				if mlog.IsDebug() {
					recordDebugDataDirection(i, debugBoneNames, direction, originalDeltas,
						debugTargetOriginal, debugTypeInitial, debugPositions, debugRotations)
				}

				continue
			}

			// 足IK親などで動かされている場合もあるので、足IKのグローバル位置で判定
			// 足首だと、足IKの結果で少し沈んだ時とかに誤判定してしまう
			legBoneName := pmx.LEG_IK.StringFromDirection(direction)
			nowLegIkPosition := originalDeltas.Bones.GetByName(legBoneName).FilledGlobalPosition()
			prevLegIkPosition := originalAllDeltas[i-1].Bones.GetByName(legBoneName).FilledGlobalPosition()

			if nowLegIkPosition.Distance(prevLegIkPosition) < 0.005 {
				fixIkFlags[d][i] = true

				if mlog.IsDebug() {
					recordDebugDataDirection(i, debugBoneNames, direction, originalDeltas,
						debugTargetOriginal, debugTypeInitial, debugPositions, debugRotations)
				}
			} else {
				if mlog.IsDebug() {
					recordDebugDataDirection(i, debugBoneNames, direction, originalDeltas,
						debugTargetSizing, debugTypeInitial, debugPositions, debugRotations)
				}
			}
		}

		if i > 0 && i%1000 == 0 {
			processLog("足補正17", sizingSet.Index, i, len(allFrames))
		}
	}

	if mlog.IsDebug() {
		outputDebugData(allFrames, debugBoneNames, verboseMotionKey, sizingSet.OutputMotionPath, sizingSet.SizingConfigModel, debugPositions, debugRotations)
	}

	processLog("足補正18", sizingSet.Index, 0, 0)

	for d, direction := range directions {
		{
			boneName := pmx.LEG_IK.StringFromDirection(direction)
			sizingSet.OutputMotion.BoneFrames.Get(boneName).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
				if fixIkFlags[d][int(frame)] {
					// 固定されている場合、前のキーフレを引き継ぐ
					prevFrame := sizingSet.OutputMotion.BoneFrames.Get(boneName).PrevFrame(frame)
					prevBf := sizingSet.OutputMotion.BoneFrames.Get(boneName).Get(prevFrame)
					bf.Position = prevBf.FilledPosition().Copy()
					sizingSet.OutputMotion.BoneFrames.Get(boneName).Update(bf)
				}

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
	sizingSet.OriginalLegIkParentBone(pmx.BONE_DIRECTION_LEFT)
	sizingSet.OriginalLegIkParentBone(pmx.BONE_DIRECTION_RIGHT)
	sizingSet.SizingLegIkParentBone(pmx.BONE_DIRECTION_LEFT)
	sizingSet.SizingLegIkParentBone(pmx.BONE_DIRECTION_RIGHT)

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
			{CheckFunk: sizingSet.OriginalAnkleBone, BoneName: pmx.ANKLE},
			{CheckFunk: sizingSet.OriginalToeIkBone, BoneName: pmx.TOE_IK},
			{CheckFunk: sizingSet.OriginalHeelBone, BoneName: pmx.HEEL},
			{CheckFunk: sizingSet.OriginalToeTailBone, BoneName: pmx.TOE_T},
			{CheckFunk: sizingSet.OriginalToePBone, BoneName: pmx.TOE_P},
			{CheckFunk: sizingSet.OriginalToeCBone, BoneName: pmx.TOE_C},
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
			{CheckFunk: sizingSet.SizingAnkleBone, BoneName: pmx.ANKLE},
			{CheckFunk: sizingSet.SizingToeIkBone, BoneName: pmx.TOE_IK},
			{CheckFunk: sizingSet.SizingHeelBone, BoneName: pmx.HEEL},
			{CheckFunk: sizingSet.SizingToeTailBone, BoneName: pmx.TOE_T},
			{CheckFunk: sizingSet.SizingToePBone, BoneName: pmx.TOE_P},
			{CheckFunk: sizingSet.SizingToeCBone, BoneName: pmx.TOE_C},
		},
	)
}
