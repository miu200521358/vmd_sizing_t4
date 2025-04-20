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
	if err := su.checkBonesForSizingLeg(sizingSet); err != nil {
		return false, err
	}

	allFrames := mmath.IntRanges(int(originalMotion.MaxFrame()) + 1)
	blockSize, _ := miter.GetBlockSize(len(allFrames) * sizingSetCount)

	// [焼き込み] -----------------------

	// 元モデルのデフォーム結果を並列処理で取得
	originalAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.OriginalConfigModel,
		originalMotion, sizingSet, true, sizingSet.OriginalConfigModel.Bones.GetStandardBoneNames(), "足補正01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	// TOE_IK キーフレームのリセット
	sizingProcessMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.TOE_IK.Left()))
	sizingProcessMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.TOE_IK.Right()))

	// サイジング先モデルに対して FK 焼き込み処理
	if err := su.updateLegFK(sizingSet, sizingProcessMotion, originalAllDeltas); err != nil {
		return false, err
	}

	if mlog.IsDebug() {
		su.insertIKFrames(sizingSet, sizingProcessMotion, false)
		outputVerboseMotion("足01", sizingSet.OutputMotionPath, sizingProcessMotion)
	}

	incrementCompletedCount()

	// [下半身] -----------------------

	// 先モデルの足中心までのデフォーム結果を並列処理で取得
	sizingLowerAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel,
		sizingProcessMotion, sizingSet, true, trunk_lower_bone_names, "足補正01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	// 下半身補正を実施
	lowerRotations, err := su.calculateAdjustedLower(sizingSet, allFrames, blockSize,
		originalAllDeltas, sizingLowerAllDeltas, sizingProcessMotion, "足02")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	// 下半身回転をサイジング先モーションに反映
	su.updateLower(sizingSet, allFrames, sizingProcessMotion, lowerRotations)

	if mlog.IsDebug() {
		outputVerboseMotion("足03", sizingSet.OutputMotionPath, sizingProcessMotion)
	}

	incrementCompletedCount()

	// [足IK] -----------------------

	// 先モデルの足のIK OFF状態でのデフォーム結果を並列処理で取得
	sizingLegIkAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel,
		sizingProcessMotion, sizingSet, false, all_lower_leg_bone_names, "足補正01")
	if err != nil {
		return false, err
	}

	// 足IK 補正処理
	leftLegIkPositions, rightLegIkPositions, leftLegRotations, rightLegRotations, err :=
		su.calculateAdjustedLegIK(sizingSet, allFrames, blockSize, moveScale,
			originalAllDeltas, sizingLegIkAllDeltas, sizingProcessMotion, "足04")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	// 足IKの位置と足の角度 をサイジング先モーションに反映
	su.updateLegIkAndFk(sizingSet, allFrames, sizingProcessMotion,
		leftLegIkPositions, rightLegIkPositions, leftLegRotations, rightLegRotations)

	if mlog.IsDebug() {
		outputVerboseMotion("足05", sizingSet.OutputMotionPath, sizingProcessMotion)
	}

	incrementCompletedCount()

	// [センター] -----------------------

	// 先モデルのIF ONデフォーム結果を並列処理で取得
	sizingCenterAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel,
		sizingProcessMotion, sizingSet, false, all_lower_leg_bone_names, "足補正01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	// センター・グルーブ補正を実施
	centerPositions, groovePositions, err := su.calculateAdjustedCenter(sizingSet, allFrames, blockSize, moveScale,
		originalAllDeltas, sizingCenterAllDeltas, sizingProcessMotion, "足06")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	// センター・グルーブ位置をサイジング先モーションに反映
	su.updateCenter(sizingSet, allFrames, sizingProcessMotion, centerPositions, groovePositions)

	if mlog.IsDebug() {
		outputVerboseMotion("足07", sizingSet.OutputMotionPath, sizingProcessMotion)
	}

	incrementCompletedCount()

	// // 先モデルのデフォーム結果を並列処理で取得
	// sizingOffAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel, sizingProcessMotion, sizingSet, false, all_lower_leg_bone_names, "足補正01")
	// if err != nil {
	// 	return false, err
	// }

	// incrementCompletedCount()

	// // 足IKの継承
	// su.inheritanceLegIk(sizingSet, allFrames, leftLegIkPositions, rightLegIkPositions, "足03")

	// incrementCompletedCount()

	// // 足IK 補正処理
	// leftLegAnkleIdealPositions, rightLegAnkleIdealPositions, err := calculateAdjustedLegIK(
	// 	sizingSet, allFrames, blockSize, moveScale, originalAllDeltas, sizingOffAllDeltas, sizingProcessMotion, incrementCompletedCount,
	// )
	// if err != nil {
	// 	return false, err
	// }

	// // 足FK 再計算（IK ON状態）
	// if err := calculateAdjustedLegFK(sizingSet, allFrames, blockSize,
	// 	sizingOffAllDeltas, leftLegAnkleIdealPositions, rightLegAnkleIdealPositions,
	// 	sizingProcessMotion, incrementCompletedCount); err != nil {
	// 	return false, err
	// }

	// // sizingSet.OutputMotion = sizingProcessMotion
	// // 足補正処理の結果をサイジング先モーションに反映
	// if err = updateLegResultMotion(
	// 	sizingSet, allFrames, blockSize, sizingProcessMotion,
	// 	incrementCompletedCount,
	// ); err != nil {
	// 	return false, err
	// }

	sizingSet.CompletedSizingLeg = true

	return true, nil
}

// // computeInitialGravity は、対象モデルの初期重心位置を計算します。
// func computeInitialGravity(sizingSet *domain.SizingSet, model *pmx.PmxModel,
// 	gravityVolumes map[string]float64, initialMotion *vmd.VmdMotion,
// ) (gravityGlobalPosition, gravityLocalPosition *mmath.MVec3) {
// 	allVmdDeltas, _ := computeVmdDeltas([]int{0}, 1, model, initialMotion, sizingSet, false, gravity_bone_names, "")
// 	return calcGravity(allVmdDeltas[0], gravityVolumes)
// }

// func calcGravity(vmdDeltas *delta.VmdDeltas, gravityVolumes map[string]float64) (
// 	gravityGlobalPosition, gravityLocalPosition *mmath.MVec3,
// ) {
// 	gravityGlobalPosition = mmath.NewMVec3()

// 	for boneName, volume := range gravityVolumes {
// 		// 重心計算元ボーン
// 		fromBoneDelta := vmdDeltas.Bones.GetByName(boneName)
// 		if fromBoneDelta == nil || fromBoneDelta.Bone == nil {
// 			continue
// 		}

// 		// 重心計算先ボーン
// 		toBoneName := fromBoneDelta.Bone.Config().GravityTargetBoneName.StringFromDirection(fromBoneDelta.Bone.Direction())
// 		toBoneDelta := vmdDeltas.Bones.GetByName(toBoneName)
// 		if toBoneDelta == nil || toBoneDelta.Bone == nil {
// 			continue
// 		}
// 		gravityGlobalPosition.Add(toBoneDelta.FilledGlobalPosition().Added(
// 			fromBoneDelta.FilledGlobalPosition()).MuledScalar(0.5 * volume))
// 	}

// 	// 体幹中心から見たローカル位置
// 	gravityLocalPosition = vmdDeltas.Bones.GetByName(pmx.TRUNK_ROOT.String()).FilledGlobalMatrix().Inverted().MulVec3(gravityGlobalPosition)
// 	return gravityGlobalPosition, gravityLocalPosition
// }

// updateLegFK は、デフォーム結果から FK 回転をサイジング先モーションに焼き込みます。
func (su *SizingLegUsecase) updateLegFK(
	sizingSet *domain.SizingSet, sizingProcessMotion *vmd.VmdMotion, allDeltas []*delta.VmdDeltas,
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

func (su *SizingLegUsecase) createLowerIkBone(sizingSet *domain.SizingSet) *pmx.Bone {
	lowerBone := sizingSet.SizingLowerBone()
	legCenterBone := sizingSet.SizingLegCenterBone()

	// 下半身IK
	ikBone := pmx.NewBoneByName(fmt.Sprintf("%s%sIk", pmx.MLIB_PREFIX, lowerBone.Name()))
	ikBone.Position = legCenterBone.Position.Copy()
	ikBone.Ik = pmx.NewIk()
	ikBone.Ik.BoneIndex = legCenterBone.Index()
	ikBone.Ik.LoopCount = 100
	ikBone.Ik.UnitRotation = &mmath.MVec3{X: 0.1, Y: 0.0, Z: 0.0}
	ikBone.Ik.Links = make([]*pmx.IkLink, 0)
	for _, parentBoneIndex := range legCenterBone.ParentBoneIndexes {
		link := pmx.NewIkLink()
		link.BoneIndex = parentBoneIndex
		if parentBoneIndex != lowerBone.Index() {
			// 下半身以外は動かさない
			link.AngleLimit = true
		}
		ikBone.Ik.Links = append(ikBone.Ik.Links, link)

		if parentBoneIndex == lowerBone.Index() {
			// 体幹根元までいったら終了
			break
		}
	}

	return ikBone
}

// calculateAdjustedCenter は、センターおよびグルーブの位置補正を並列処理で計算します。
func (su *SizingLegUsecase) calculateAdjustedLower(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int,
	originalAllDeltas, sizingAllDeltas []*delta.VmdDeltas, sizingProcessMotion *vmd.VmdMotion, verboseMotionName string,
) (sizingLowerRotations []*mmath.MQuaternion, err error) {
	originalLowerPositions := make([]*mmath.MVec3, len(allFrames))
	originalLowerRotations := make([]*mmath.MQuaternion, len(allFrames))

	originalLeftLegPositions := make([]*mmath.MVec3, len(allFrames))
	originalRightLegPositions := make([]*mmath.MVec3, len(allFrames))

	sizingLegCenterIdealPositions := make([]*mmath.MVec3, len(allFrames))
	sizingLeftLegIdealPositions := make([]*mmath.MVec3, len(allFrames))
	sizingRightLegIdealPositions := make([]*mmath.MVec3, len(allFrames))

	sizingLegCenterPositions := make([]*mmath.MVec3, len(allFrames))
	sizingLeftLegPositions := make([]*mmath.MVec3, len(allFrames))
	sizingRightLegPositions := make([]*mmath.MVec3, len(allFrames))

	sizingLowerRootInitialPositions := make([]*mmath.MVec3, len(allFrames))
	sizingLegCenterInitialPositions := make([]*mmath.MVec3, len(allFrames))
	sizingLeftLegInitialPositions := make([]*mmath.MVec3, len(allFrames))
	sizingRightLegInitialPositions := make([]*mmath.MVec3, len(allFrames))
	sizingLowerInitialPositions := make([]*mmath.MVec3, len(allFrames))
	sizingLowerInitialRotations := make([]*mmath.MQuaternion, len(allFrames))
	sizingLowerPositions := make([]*mmath.MVec3, len(allFrames))
	sizingLowerRotations = make([]*mmath.MQuaternion, len(allFrames))

	lowerIkBone := su.createLowerIkBone(sizingSet)

	// 下半身根元から足中心の傾き
	originalLegCenterFromLowerRootDiff := sizingSet.OriginalLegCenterBone().Position.Subed(
		sizingSet.OriginalLowerRootBone().Position)
	sizingLegCenterFromLowerRootDiff := sizingSet.SizingLegCenterBone().Position.Subed(
		sizingSet.SizingLowerRootBone().Position)

	// 真下から足中心までの傾き
	originalLegSlope := mmath.NewMQuaternionRotate(mmath.MVec3UnitYNeg, originalLegCenterFromLowerRootDiff.Normalized())
	sizingLegSlope := mmath.NewMQuaternionRotate(mmath.MVec3UnitYNeg, sizingLegCenterFromLowerRootDiff.Normalized())

	// 下半身根元から足までの差分
	originalLeftLegFromLowerRootDiff :=
		sizingSet.OriginalLeftLegBone().Position.Subed(sizingSet.OriginalLowerRootBone().Position)
	originalRightLegFromLowerRootDiff :=
		sizingSet.OriginalRightLegBone().Position.Subed(sizingSet.OriginalLowerRootBone().Position)
	sizingLeftLegFromLowerRootDiff :=
		sizingSet.SizingLeftLegBone().Position.Subed(sizingSet.SizingLowerRootBone().Position)
	sizingRightLegFromLowerRootDiff :=
		sizingSet.SizingRightLegBone().Position.Subed(sizingSet.SizingLowerRootBone().Position)

	// 下半身根元から足までの真っ直ぐにしたときの長さ差
	originalLegCenterVerticalDiff := originalLegSlope.Inverted().MulVec3(originalLegCenterFromLowerRootDiff).Truncate(1e-3)
	originalLeftLegVerticalDiff := originalLegSlope.Inverted().MulVec3(originalLeftLegFromLowerRootDiff).Truncate(1e-3)
	originalRightLegVerticalDiff := originalLegSlope.Inverted().MulVec3(originalRightLegFromLowerRootDiff).Truncate(1e-3)

	sizingLegCenterVerticalDiff := sizingLegSlope.Inverted().MulVec3(sizingLegCenterFromLowerRootDiff).Truncate(1e-3)
	sizingLeftLegVerticalDiff := sizingLegSlope.Inverted().MulVec3(sizingLeftLegFromLowerRootDiff).Truncate(1e-3)
	sizingRightLegVerticalDiff := sizingLegSlope.Inverted().MulVec3(sizingRightLegFromLowerRootDiff).Truncate(1e-3)

	legCenterFromLowerRootScale := sizingLegCenterVerticalDiff.Dived(originalLegCenterVerticalDiff).Effective().One()
	leftLegFromLowerRootScale := sizingLeftLegVerticalDiff.Dived(originalLeftLegVerticalDiff).Effective().One()
	rightLegFromLowerRootScale := sizingRightLegVerticalDiff.Dived(originalRightLegVerticalDiff).Effective().One()

	err = miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			originalLowerRootDelta := originalAllDeltas[index].Bones.GetByName(pmx.LOWER_ROOT.String())
			originalLegCenterDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG_CENTER.String())
			originalLeftLegDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG.Left())
			originalRightLegDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG.Right())

			// 下半身根元から見た足ボーンのローカル位置
			originalLegCenterLocalPosition := originalLowerRootDelta.FilledGlobalMatrix().Inverted().MulVec3(originalLegCenterDelta.FilledGlobalPosition())
			originalLeftLegLocalPosition := originalLowerRootDelta.FilledGlobalMatrix().Inverted().MulVec3(originalLeftLegDelta.FilledGlobalPosition())
			originalRightLegLocalPosition := originalLowerRootDelta.FilledGlobalMatrix().Inverted().MulVec3(originalRightLegDelta.FilledGlobalPosition())

			// 真っ直ぐにしたときのローカル位置
			originalLegCenterVerticalLocalPosition := originalLegSlope.Inverted().MulVec3(originalLegCenterLocalPosition).Truncate(1e-3)
			originalLeftLegVerticalLocalPosition := originalLegSlope.Inverted().MulVec3(originalLeftLegLocalPosition).Truncate(1e-3)
			originalRightLegVerticalLocalPosition := originalLegSlope.Inverted().MulVec3(originalRightLegLocalPosition).Truncate(1e-3)

			// スケール差を考慮した先の足ボーンのローカル位置
			sizingLegCenterVerticalLocalPosition := originalLegCenterVerticalLocalPosition.Muled(legCenterFromLowerRootScale)
			sizingLeftLegVerticalLocalPosition := originalLeftLegVerticalLocalPosition.Muled(leftLegFromLowerRootScale)
			sizingRightLegVerticalLocalPosition := originalRightLegVerticalLocalPosition.Muled(rightLegFromLowerRootScale)

			sizingLegCenterLocalPosition := sizingLegSlope.Inverted().MulVec3(sizingLegCenterVerticalLocalPosition)
			sizingLeftLegLocalPosition := sizingLegSlope.Inverted().MulVec3(sizingLeftLegVerticalLocalPosition)
			sizingRightLegLocalPosition := sizingLegSlope.Inverted().MulVec3(sizingRightLegVerticalLocalPosition)

			sizingLowerRootDelta := sizingAllDeltas[index].Bones.GetByName(pmx.LOWER_ROOT.String())

			sizingLegCenterGlobalPosition := sizingLowerRootDelta.FilledGlobalMatrix().MulVec3(sizingLegCenterLocalPosition)
			sizingLeftLegIdealGlobalPosition := sizingLowerRootDelta.FilledGlobalMatrix().MulVec3(sizingLeftLegLocalPosition)
			sizingRightLegIdealGlobalPosition := sizingLowerRootDelta.FilledGlobalMatrix().MulVec3(sizingRightLegLocalPosition)

			if mlog.IsDebug() {
				originalLowerDelta := originalAllDeltas[index].Bones.GetByName(pmx.LOWER.String())
				originalLowerPositions[index] = originalLowerDelta.FilledGlobalPosition()
				originalLowerRotations[index] = originalLowerDelta.FilledFrameRotation()

				originalLeftLegPositions[index] = originalLeftLegDelta.FilledGlobalPosition().Copy()
				originalRightLegPositions[index] = originalRightLegDelta.FilledGlobalPosition().Copy()

				sizingLowerRootInitialPositions[index] = sizingLowerRootDelta.FilledGlobalPosition().Copy()
				sizingLegCenterInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(
					pmx.LEG_CENTER.String()).FilledGlobalPosition().Copy()
				sizingLeftLegInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(
					pmx.LEG.Left()).FilledGlobalPosition().Copy()
				sizingRightLegInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(
					pmx.LEG.Right()).FilledGlobalPosition().Copy()

				sizingLegCenterIdealPositions[index] = sizingLegCenterGlobalPosition.Copy()
				sizingLeftLegIdealPositions[index] = sizingLeftLegIdealGlobalPosition.Copy()
				sizingRightLegIdealPositions[index] = sizingRightLegIdealGlobalPosition.Copy()

				sizingLowerDelta := sizingAllDeltas[index].Bones.GetByName(pmx.LOWER.String())
				sizingLowerInitialPositions[index] = sizingLowerDelta.FilledGlobalPosition().Copy()
				sizingLowerInitialRotations[index] = sizingLowerDelta.FilledFrameRotation()
			}

			// IK解決
			sizingLowerDeltas := deform.DeformIks(sizingSet.SizingConfigModel, sizingProcessMotion,
				sizingAllDeltas[index], float32(data),
				[]*pmx.Bone{lowerIkBone},
				[]*pmx.Bone{sizingSet.SizingLegCenterBone()},
				[]*mmath.MVec3{sizingLegCenterGlobalPosition},
				trunk_lower_bone_names, 1, false, false)

			sizingLowerDelta := sizingLowerDeltas.Bones.GetByName(pmx.LOWER.String())
			sizingLowerRotations[index] = sizingLowerDelta.FilledFrameRotation()

			if mlog.IsDebug() {
				sizingLowerPositions[index] = sizingLowerDelta.FilledGlobalPosition().Copy()

				sizingLegCenterPositions[index] = sizingLowerDeltas.Bones.GetByName(pmx.LEG_CENTER.String()).FilledGlobalPosition().Copy()
				sizingLeftLegPositions[index] = sizingLowerDeltas.Bones.GetByName(pmx.LEG.Left()).FilledGlobalPosition().Copy()
				sizingRightLegPositions[index] = sizingLowerDeltas.Bones.GetByName(pmx.LEG.Right()).FilledGlobalPosition().Copy()
			}

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

		for i, iFrame := range allFrames {
			frame := float32(iFrame)
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = originalLowerPositions[i]
				bf.Rotation = originalLowerRotations[i]
				motion.InsertBoneFrame("元現在下半身", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = originalLeftLegPositions[i]
				motion.InsertBoneFrame("元現在左足", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = originalRightLegPositions[i]
				motion.InsertBoneFrame("元現在右足", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = sizingLowerRootInitialPositions[i]
				motion.InsertBoneFrame("先現在下根元", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = sizingLegCenterInitialPositions[i]
				motion.InsertBoneFrame("先現在足中心", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = sizingLegCenterIdealPositions[i]
				motion.InsertBoneFrame("先理想足中心", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = sizingLegCenterPositions[i]
				motion.InsertBoneFrame("先結果足中心", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = sizingLeftLegInitialPositions[i]
				motion.InsertBoneFrame("先現在左足", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = sizingLeftLegIdealPositions[i]
				motion.InsertBoneFrame("先理想左足", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = sizingLeftLegPositions[i]
				motion.InsertBoneFrame("先結果左足", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = sizingRightLegInitialPositions[i]
				motion.InsertBoneFrame("先現在右足", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = sizingRightLegIdealPositions[i]
				motion.InsertBoneFrame("先理想右足", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = sizingRightLegPositions[i]
				motion.InsertBoneFrame("先結果右足", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = sizingLowerInitialPositions[i]
				bf.Rotation = sizingLowerInitialRotations[i]
				motion.InsertBoneFrame("先現在下半身", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = sizingLowerPositions[i]
				bf.Rotation = sizingLowerRotations[i]
				motion.InsertBoneFrame("先結果下半身", bf)
			}
		}

		outputVerboseMotion(verboseMotionName, sizingSet.OutputMotionPath, motion)
	}

	return sizingLowerRotations, nil
}

// updateLower は、補正した下半身回転をサイジング先モーションに反映します。
func (su *SizingLegUsecase) updateLower(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	lowerRotations []*mmath.MQuaternion,
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
	}
}

func (su *SizingLegUsecase) createFullLegIkBone(sizingSet *domain.SizingSet, direction pmx.BoneDirection) (
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
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, moveScale *mmath.MVec3,
	originalAllDeltas, sizingAllDeltas []*delta.VmdDeltas, sizingProcessMotion *vmd.VmdMotion, verboseMotionKey string,
) (leftLegIkPositions, rightLegIkPositions []*mmath.MVec3,
	leftLegRotations, rightLegRotations []*mmath.MQuaternion, err error) {

	leftLegIkPositions = make([]*mmath.MVec3, len(allFrames))
	leftLegRotations = make([]*mmath.MQuaternion, len(allFrames))
	rightLegIkPositions = make([]*mmath.MVec3, len(allFrames))
	rightLegRotations = make([]*mmath.MQuaternion, len(allFrames))

	originalLeftToeTailDInitialPositions := make([]*mmath.MVec3, len(allFrames))
	originalRightToeTailDInitialPositions := make([]*mmath.MVec3, len(allFrames))
	leftToeTailDIdealPositions := make([]*mmath.MVec3, len(allFrames))
	rightToeTailDIdealPositions := make([]*mmath.MVec3, len(allFrames))

	leftLegInitialPositions := make([]*mmath.MVec3, len(allFrames))
	leftKneeInitialPositions := make([]*mmath.MVec3, len(allFrames))
	leftAnkleInitialPositions := make([]*mmath.MVec3, len(allFrames))
	leftKneeDInitialPositions := make([]*mmath.MVec3, len(allFrames))
	leftAnkleDInitialPositions := make([]*mmath.MVec3, len(allFrames))
	leftHeelDInitialPositions := make([]*mmath.MVec3, len(allFrames))
	leftToeTailDInitialPositions := make([]*mmath.MVec3, len(allFrames))
	leftToePDInitialPositions := make([]*mmath.MVec3, len(allFrames))

	rightLegInitialPositions := make([]*mmath.MVec3, len(allFrames))
	rightKneeInitialPositions := make([]*mmath.MVec3, len(allFrames))
	rightAnkleInitialPositions := make([]*mmath.MVec3, len(allFrames))
	rightKneeDInitialPositions := make([]*mmath.MVec3, len(allFrames))
	rightAnkleDInitialPositions := make([]*mmath.MVec3, len(allFrames))
	rightHeelDInitialPositions := make([]*mmath.MVec3, len(allFrames))
	rightToeTailDInitialPositions := make([]*mmath.MVec3, len(allFrames))
	rightToePDInitialPositions := make([]*mmath.MVec3, len(allFrames))

	leftLegResultPositions := make([]*mmath.MVec3, len(allFrames))
	leftKneeResultPositions := make([]*mmath.MVec3, len(allFrames))
	leftAnkleResultPositions := make([]*mmath.MVec3, len(allFrames))
	leftKneeDResultPositions := make([]*mmath.MVec3, len(allFrames))
	leftAnkleDResultPositions := make([]*mmath.MVec3, len(allFrames))
	leftToeTailDResultPositions := make([]*mmath.MVec3, len(allFrames))
	leftHeelDResultPositions := make([]*mmath.MVec3, len(allFrames))
	leftToePDResultPositions := make([]*mmath.MVec3, len(allFrames))

	rightLegResultPositions := make([]*mmath.MVec3, len(allFrames))
	rightKneeResultPositions := make([]*mmath.MVec3, len(allFrames))
	rightAnkleResultPositions := make([]*mmath.MVec3, len(allFrames))
	rightKneeDResultPositions := make([]*mmath.MVec3, len(allFrames))
	rightAnkleDResultPositions := make([]*mmath.MVec3, len(allFrames))
	rightToeTailDResultPositions := make([]*mmath.MVec3, len(allFrames))
	rightHeelDResultPositions := make([]*mmath.MVec3, len(allFrames))
	rightToePDResultPositions := make([]*mmath.MVec3, len(allFrames))

	sizingLeftLegIkBone := sizingSet.SizingLeftLegIkBone()
	sizingLeftLegIkParentBone := sizingSet.SizingLeftLegIkBone().ParentBone
	sizingRightLegIkBone := sizingSet.SizingRightLegIkBone()
	sizingRightLegIkParentBone := sizingSet.SizingRightLegIkBone().ParentBone

	// 足根元から足首地面までの長さ差
	originalLeftLegLength := sizingSet.OriginalLeftLegBone().Position.Distance(
		sizingSet.OriginalLeftKneeBone().Position) +
		sizingSet.OriginalLeftKneeBone().Position.Distance(
			sizingSet.OriginalLeftAnkleBone().Position) +
		sizingSet.OriginalLeftAnkleBone().Position.Distance(
			sizingSet.OriginalLeftAnkleDGroundBone().Position)
	originalRightLegLength := sizingSet.OriginalRightLegBone().Position.Distance(
		sizingSet.OriginalRightKneeBone().Position) +
		sizingSet.OriginalRightKneeBone().Position.Distance(
			sizingSet.OriginalRightAnkleBone().Position) +
		sizingSet.OriginalRightAnkleBone().Position.Distance(
			sizingSet.OriginalRightAnkleDGroundBone().Position)
	sizingLeftLegLength := sizingSet.SizingLeftLegBone().Position.Distance(
		sizingSet.SizingLeftKneeBone().Position) +
		sizingSet.SizingLeftKneeBone().Position.Distance(
			sizingSet.SizingLeftAnkleBone().Position) +
		sizingSet.SizingLeftAnkleBone().Position.Distance(
			sizingSet.SizingLeftAnkleDGroundBone().Position)
	sizingRightLegLength := sizingSet.SizingRightLegBone().Position.Distance(
		sizingSet.SizingRightKneeBone().Position) +
		sizingSet.SizingRightKneeBone().Position.Distance(
			sizingSet.SizingRightAnkleBone().Position) +
		sizingSet.SizingRightAnkleBone().Position.Distance(
			sizingSet.SizingRightAnkleDGroundBone().Position)
	leftLegScale := sizingLeftLegLength / originalLeftLegLength
	rightLegScale := sizingRightLegLength / originalRightLegLength

	leftToeTailDIkBone := su.createFullLegIkBone(sizingSet, pmx.BONE_DIRECTION_LEFT)
	rightToeTailDIkBone := su.createFullLegIkBone(sizingSet, pmx.BONE_DIRECTION_RIGHT)

	err = miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			originalLeftLegRootDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG_ROOT.Left())
			originalRightLegRootDelta := originalAllDeltas[index].Bones.GetByName(pmx.LEG_ROOT.Right())
			originalLeftToeTailDDelta := originalAllDeltas[index].Bones.GetByName(pmx.TOE_T_D.Left())
			originalRightToeTailDDelta := originalAllDeltas[index].Bones.GetByName(pmx.TOE_T_D.Right())

			sizingLeftLegRootDelta := sizingAllDeltas[index].Bones.GetByName(pmx.LEG_ROOT.Left())
			sizingRightLegRootDelta := sizingAllDeltas[index].Bones.GetByName(pmx.LEG_ROOT.Right())

			// 足根元ボーンから見たつま先親Dまでの相対位置（角度を見たくないのでグローバルの差分で）
			originalLeftToeTailDRelativePosition := originalLeftToeTailDDelta.FilledGlobalPosition().Subed(originalLeftLegRootDelta.FilledGlobalPosition())
			originalRightToeTailDRelativePosition := originalRightToeTailDDelta.FilledGlobalPosition().Subed(originalRightLegRootDelta.FilledGlobalPosition())

			// スケール差を考慮した先のつま先PDボーンのローカル位置
			sizingLeftToeTailDRelativePosition := originalLeftToeTailDRelativePosition.MuledScalar(leftLegScale)
			sizingRightToeTailDRelativePosition := originalRightToeTailDRelativePosition.MuledScalar(rightLegScale)

			leftToeTailDIdealPosition := sizingLeftLegRootDelta.FilledGlobalPosition().Added(sizingLeftToeTailDRelativePosition)
			rightToeTailDIdealPosition := sizingRightLegRootDelta.FilledGlobalPosition().Added(sizingRightToeTailDRelativePosition)

			if mlog.IsDebug() {
				originalLeftToeTailDInitialPositions[index] = originalLeftToeTailDDelta.FilledGlobalPosition().Copy()
				originalRightToeTailDInitialPositions[index] = originalRightToeTailDDelta.FilledGlobalPosition().Copy()

				leftToeTailDIdealPositions[index] = leftToeTailDIdealPosition.Copy()
				rightToeTailDIdealPositions[index] = rightToeTailDIdealPosition.Copy()

				leftLegInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(pmx.LEG.Left()).FilledGlobalPosition().Copy()
				leftKneeInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(pmx.KNEE.Left()).FilledGlobalPosition().Copy()
				leftAnkleInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(pmx.ANKLE.Left()).FilledGlobalPosition().Copy()
				leftKneeDInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(pmx.KNEE_D.Left()).FilledGlobalPosition().Copy()
				leftAnkleDInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(pmx.ANKLE_D.Left()).FilledGlobalPosition().Copy()
				leftToeTailDInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(pmx.TOE_T_D.Left()).FilledGlobalPosition().Copy()
				leftHeelDInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(pmx.HEEL_D.Left()).FilledGlobalPosition().Copy()
				leftToePDInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(pmx.TOE_P_D.Left()).FilledGlobalPosition().Copy()

				rightLegInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(pmx.LEG.Right()).FilledGlobalPosition().Copy()
				rightKneeInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(pmx.KNEE.Right()).FilledGlobalPosition().Copy()
				rightAnkleInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(pmx.ANKLE.Right()).FilledGlobalPosition().Copy()
				rightKneeDInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(pmx.KNEE_D.Right()).FilledGlobalPosition().Copy()
				rightAnkleDInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(pmx.ANKLE_D.Right()).FilledGlobalPosition().Copy()
				rightToeTailDInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(pmx.TOE_T_D.Right()).FilledGlobalPosition().Copy()
				rightHeelDInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(pmx.HEEL_D.Right()).FilledGlobalPosition().Copy()
				rightToePDInitialPositions[index] = sizingAllDeltas[index].Bones.GetByName(pmx.TOE_P_D.Right()).FilledGlobalPosition().Copy()
			}

			// IK解決
			sizingLeftLegDeltas := deform.DeformIks(sizingSet.SizingConfigModel, sizingProcessMotion,
				sizingAllDeltas[index], float32(data),
				[]*pmx.Bone{leftToeTailDIkBone},
				[]*pmx.Bone{sizingSet.SizingLeftToeTailDBone()},
				[]*mmath.MVec3{leftToeTailDIdealPosition},
				leg_direction_bone_names[0], 1, false, false)

			sizingRightLegDeltas := deform.DeformIks(sizingSet.SizingConfigModel, sizingProcessMotion,
				sizingAllDeltas[index], float32(data),
				[]*pmx.Bone{rightToeTailDIkBone},
				[]*pmx.Bone{sizingSet.SizingRightToeTailDBone()},
				[]*mmath.MVec3{rightToeTailDIdealPosition},
				leg_direction_bone_names[1], 1, false, false)

			if mlog.IsDebug() {
				leftLegResultPositions[index] = sizingLeftLegDeltas.Bones.GetByName(pmx.LEG.Left()).FilledGlobalPosition().Copy()
				leftKneeResultPositions[index] = sizingLeftLegDeltas.Bones.GetByName(pmx.KNEE.Left()).FilledGlobalPosition().Copy()
				leftAnkleResultPositions[index] = sizingLeftLegDeltas.Bones.GetByName(pmx.ANKLE.Left()).FilledGlobalPosition().Copy()
				leftKneeDResultPositions[index] = sizingLeftLegDeltas.Bones.GetByName(pmx.KNEE_D.Left()).FilledGlobalPosition().Copy()
				leftAnkleDResultPositions[index] = sizingLeftLegDeltas.Bones.GetByName(pmx.ANKLE_D.Left()).FilledGlobalPosition().Copy()
				leftToeTailDResultPositions[index] = sizingLeftLegDeltas.Bones.GetByName(pmx.TOE_T_D.Left()).FilledGlobalPosition().Copy()
				leftHeelDResultPositions[index] = sizingLeftLegDeltas.Bones.GetByName(pmx.HEEL_D.Left()).FilledGlobalPosition().Copy()
				leftToePDResultPositions[index] = sizingLeftLegDeltas.Bones.GetByName(pmx.TOE_P_D.Left()).FilledGlobalPosition().Copy()

				rightLegResultPositions[index] = sizingRightLegDeltas.Bones.GetByName(pmx.LEG.Right()).FilledGlobalPosition().Copy()
				rightKneeResultPositions[index] = sizingRightLegDeltas.Bones.GetByName(pmx.KNEE.Right()).FilledGlobalPosition().Copy()
				rightAnkleResultPositions[index] = sizingRightLegDeltas.Bones.GetByName(pmx.ANKLE.Right()).FilledGlobalPosition().Copy()
				rightKneeDResultPositions[index] = sizingRightLegDeltas.Bones.GetByName(pmx.KNEE_D.Right()).FilledGlobalPosition().Copy()
				rightAnkleDResultPositions[index] = sizingRightLegDeltas.Bones.GetByName(pmx.ANKLE_D.Right()).FilledGlobalPosition().Copy()
				rightToeTailDResultPositions[index] = sizingRightLegDeltas.Bones.GetByName(pmx.TOE_T_D.Right()).FilledGlobalPosition().Copy()
				rightHeelDResultPositions[index] = sizingRightLegDeltas.Bones.GetByName(pmx.HEEL_D.Right()).FilledGlobalPosition().Copy()
				rightToePDResultPositions[index] = sizingRightLegDeltas.Bones.GetByName(pmx.TOE_P_D.Right()).FilledGlobalPosition().Copy()
			}

			leftAnkleDelta := sizingLeftLegDeltas.Bones.GetByName(pmx.ANKLE.Left())
			rightAnkleDelta := sizingRightLegDeltas.Bones.GetByName(pmx.ANKLE.Right())

			// 足IKの親からみた足IKデフォルト位置からみた現在の足首のローカル位置
			sizingLeftLegIkParentDelta := sizingLeftLegDeltas.Bones.Get(sizingLeftLegIkParentBone.Index())
			sizingLeftLegIkMatrix := sizingLeftLegIkParentDelta.FilledGlobalMatrix().Translated(sizingLeftLegIkBone.ParentRelativePosition)
			leftLegIkPositions[index] = sizingLeftLegIkMatrix.Inverted().MulVec3(leftAnkleDelta.FilledGlobalPosition())

			sizingRightLegIkParentDelta := sizingRightLegDeltas.Bones.Get(sizingRightLegIkParentBone.Index())
			sizingRightLegIkMatrix := sizingRightLegIkParentDelta.FilledGlobalMatrix().Translated(sizingRightLegIkBone.ParentRelativePosition)
			rightLegIkPositions[index] = sizingRightLegIkMatrix.Inverted().MulVec3(rightAnkleDelta.FilledGlobalPosition())

			leftLegRotations[index] = sizingLeftLegDeltas.Bones.GetByName(pmx.LEG.Left()).FilledFrameRotation().Copy()
			rightLegRotations[index] = sizingRightLegDeltas.Bones.GetByName(pmx.LEG.Right()).FilledFrameRotation().Copy()

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正07", sizingSet.Index, iterIndex, allCount)
		})
	if err != nil {
		return nil, nil, nil, nil, err
	}

	if mlog.IsDebug() {
		motion := vmd.NewVmdMotion("")

		for i, iFrame := range allFrames {
			frame := float32(iFrame)
			for _, v := range [][]any{
				{originalLeftToeTailDInitialPositions, "元今左先TD"},
				{originalRightToeTailDInitialPositions, "元今右先TD"},
				{leftToeTailDIdealPositions, "先理左先TD"},
				{rightToeTailDIdealPositions, "先理右先TD"},
				{leftLegInitialPositions, "先今左足"},
				{leftKneeInitialPositions, "先今左膝"},
				{leftAnkleInitialPositions, "先今左足首"},
				{leftKneeDInitialPositions, "先今左膝D"},
				{leftAnkleDInitialPositions, "先今左足首D"},
				{leftHeelDInitialPositions, "先今左踵D"},
				{leftToeTailDInitialPositions, "先今左先D"},
				{leftToePDInitialPositions, "先今左先PD"},
				{rightLegInitialPositions, "先今右足"},
				{rightKneeInitialPositions, "先今右膝"},
				{rightAnkleInitialPositions, "先今右足首"},
				{rightKneeDInitialPositions, "先今右膝D"},
				{rightAnkleDInitialPositions, "先今右足首D"},
				{rightHeelDInitialPositions, "先今右踵D"},
				{rightToeTailDInitialPositions, "先今右先D"},
				{rightToePDInitialPositions, "先今右先PD"},
				{leftLegResultPositions, "先結左足"},
				{leftKneeResultPositions, "先結左膝"},
				{leftAnkleResultPositions, "先結左足首"},
				{leftKneeDResultPositions, "先結左膝D"},
				{leftAnkleDResultPositions, "先結左足首D"},
				{leftToeTailDResultPositions, "先結左先D"},
				{leftHeelDResultPositions, "先結左踵D"},
				{leftToePDResultPositions, "先結左先PD"},
				{rightLegResultPositions, "先結右足"},
				{rightKneeResultPositions, "先結右膝"},
				{rightAnkleResultPositions, "先結右足首"},
				{rightKneeDResultPositions, "先結右膝D"},
				{rightAnkleDResultPositions, "先結右足首D"},
				{rightToeTailDResultPositions, "先結右先D"},
				{rightHeelDResultPositions, "先結右踵D"},
				{rightToePDResultPositions, "先結右先PD"},
			} {
				positions := v[0].([]*mmath.MVec3)
				boneName := v[1].(string)

				bf := vmd.NewBoneFrame(frame)
				bf.Position = positions[i]
				motion.InsertBoneFrame(boneName, bf)
			}
		}

		outputVerboseMotion(verboseMotionKey, sizingSet.OutputMotionPath, motion)
	}

	return leftLegIkPositions, rightLegIkPositions, leftLegRotations, rightLegRotations, nil
}

// updateLegIkAndFk は、計算済みの足IK 補正位置と回転値をモーションデータに反映させます。
func (su *SizingLegUsecase) updateLegIkAndFk(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	leftLegIkPositions, rightLegIkPositions []*mmath.MVec3,
	leftLegRotations, rightLegRotations []*mmath.MQuaternion,
) {
	for i, iFrame := range allFrames {
		frame := float32(iFrame)

		{
			bf := sizingProcessMotion.BoneFrames.Get(pmx.LEG_IK.Left()).Get(frame)
			bf.Position = leftLegIkPositions[i]
			sizingProcessMotion.InsertBoneFrame(pmx.LEG_IK.Left(), bf)
		}
		{
			bf := sizingProcessMotion.BoneFrames.Get(pmx.LEG_IK.Right()).Get(frame)
			bf.Position = rightLegIkPositions[i]
			sizingProcessMotion.InsertBoneFrame(pmx.LEG_IK.Right(), bf)
		}
		{
			bf := sizingProcessMotion.BoneFrames.Get(pmx.LEG.Left()).Get(frame)
			bf.Rotation = leftLegRotations[i]
			sizingProcessMotion.InsertBoneFrame(pmx.LEG.Left(), bf)
		}
		{
			bf := sizingProcessMotion.BoneFrames.Get(pmx.LEG.Right()).Get(frame)
			bf.Rotation = rightLegRotations[i]
			sizingProcessMotion.InsertBoneFrame(pmx.LEG.Right(), bf)
		}

		if i > 0 && i%1000 == 0 {
			processLog("足補正08", sizingSet.Index, i, len(allFrames))
		}
	}
}

// calculateAdjustedCenter は、センターおよびグルーブの位置補正を並列処理で計算します。
func (su *SizingLegUsecase) calculateAdjustedCenter(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, moveScale *mmath.MVec3,
	originalAllDeltas, sizingAllDeltas []*delta.VmdDeltas, sizingProcessMotion *vmd.VmdMotion, verboseMotionName string,
) (centerPositions, groovePositions []*mmath.MVec3, err error) {
	centerPositions = make([]*mmath.MVec3, len(allFrames))
	groovePositions = make([]*mmath.MVec3, len(allFrames))

	originalTrunkRootPositions := make([]*mmath.MVec3, len(allFrames))
	sizingTrunkRootPositions := make([]*mmath.MVec3, len(allFrames))
	sizingTrunkRootIdealPositions := make([]*mmath.MVec3, len(allFrames))

	isActiveGroove := false
	sizingProcessMotion.BoneFrames.Get(pmx.GROOVE.String()).ForEach(func(frame float32, bf *vmd.BoneFrame) bool {
		if !mmath.NearEquals(bf.FilledPosition().Y, 0.0, 1e-3) {
			isActiveGroove = true
			return false
		}
		return true
	})

	originalCenterParentBone := sizingSet.OriginalCenterBone().ParentBone
	sizingCenterParentBone := sizingSet.SizingCenterBone().ParentBone

	// 先の初期姿勢におけるセンターの親から見た体幹中心のローカル位置
	sizingTrunkRootInitialPosition := sizingSet.SizingTrunkRootBone().Position.Subed(sizingCenterParentBone.Position)

	err = miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			// 元の体幹中心
			originalTrunkRootDelta := originalAllDeltas[index].Bones.GetByName(pmx.TRUNK_ROOT.String())
			// 元のセンター親
			originalCenterParentDelta := originalAllDeltas[index].Bones.GetByName(originalCenterParentBone.Name())
			// センターの親から見た元の体幹中心のローカル位置
			originalTrunkRootLocalPosition := originalCenterParentDelta.FilledGlobalMatrix().Inverted().MulVec3(originalTrunkRootDelta.FilledGlobalPosition())

			// 先の体幹中心
			sizingTrunkRootDelta := sizingAllDeltas[index].Bones.GetByName(pmx.TRUNK_ROOT.String())
			// 先のセンター親
			sizingCenterParentDelta := sizingAllDeltas[index].Bones.GetByName(sizingCenterParentBone.Name())

			// 先の理想体幹中心(元の体幹中心*比率)
			sizingTrunkRootIdealLocalPosition := originalTrunkRootLocalPosition.Muled(moveScale)
			// 初期姿勢からの差分
			sizingLegCenterDiff := sizingTrunkRootIdealLocalPosition.Subed(sizingTrunkRootInitialPosition)

			centerPositions[index] = sizingLegCenterDiff.Copy()

			if isActiveGroove {
				groovePositions[index] = &mmath.MVec3{X: 0.0, Y: sizingLegCenterDiff.Y, Z: 0.0}
				centerPositions[index].Y = 0.0
			}

			if mlog.IsDebug() {
				originalTrunkRootPositions[index] = originalTrunkRootDelta.FilledGlobalPosition()
				sizingTrunkRootPositions[index] = sizingTrunkRootDelta.FilledGlobalPosition()
				sizingTrunkRootIdealPositions[index] = sizingCenterParentDelta.FilledGlobalMatrix().MulVec3(sizingTrunkRootIdealLocalPosition)
			}

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正05", sizingSet.Index, iterIndex, allCount)
		})
	if err != nil {
		return nil, nil, err
	}

	if mlog.IsDebug() {
		motion := vmd.NewVmdMotion("")

		for i, iFrame := range allFrames {
			frame := float32(iFrame)
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = originalTrunkRootPositions[i]
				motion.InsertBoneFrame("元体幹中心", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = sizingTrunkRootPositions[i]
				motion.InsertBoneFrame("先体幹中心", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = sizingTrunkRootIdealPositions[i]
				motion.InsertBoneFrame("先理想体幹中心", bf)
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

	return centerPositions, groovePositions, nil
}

// updateCenter は、計算したセンター・グルーブ位置をサイジング先モーションに反映します。
func (su *SizingLegUsecase) updateCenter(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	centerPositions, groovePositions []*mmath.MVec3,
) {
	for i, iFrame := range allFrames {
		frame := float32(iFrame)
		sizingCenterBf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingCenterBone().Name()).Get(frame)
		sizingCenterBf.Position = centerPositions[i]
		sizingProcessMotion.InsertBoneFrame(sizingSet.SizingCenterBone().Name(), sizingCenterBf)

		if sizingSet.SizingGrooveVanillaBone() != nil {
			sizingGrooveBf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingGrooveBone().Name()).Get(frame)
			sizingGrooveBf.Position = groovePositions[i]
			sizingProcessMotion.InsertBoneFrame(sizingSet.SizingGrooveBone().Name(), sizingGrooveBf)
		}

		if i > 0 && i%1000 == 0 {
			processLog("足補正06", sizingSet.Index, i, len(allFrames))
		}
	}
}

func (su *SizingLegUsecase) updateLegResultMotion(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, sizingProcessMotion *vmd.VmdMotion,
	incrementCompletedCount func(),
) error {
	// 足補正処理の結果をサイジング先モーションに反映
	sizingModel := sizingSet.SizingConfigModel
	outputMotion := sizingSet.OutputMotion

	activeFrames := getFrames(outputMotion, all_lower_leg_bone_names)

	// TOE_IK キーフレームのリセット
	outputMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.TOE_IK.Left()))
	outputMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.TOE_IK.Right()))

	// 足系はあるボーンだけ上書きする
	for _, boneName := range []string{
		pmx.CENTER.String(), pmx.GROOVE.String(), pmx.LOWER.String(),
		pmx.LEG_IK.Left(), pmx.LEG.Left(), pmx.KNEE.Left(), pmx.ANKLE.Left(),
		pmx.LEG_IK.Right(), pmx.LEG.Right(), pmx.KNEE.Right(), pmx.ANKLE.Right(),
	} {
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

	// 中間キーフレのズレをチェック
	kneeThreshold := 0.3
	ankleThreshold := 0.2
	toeThreshold := 0.1

	err := miter.IterParallelByList(directions, 1, 1,
		func(dIndex int, direction pmx.BoneDirection) error {
			for tIndex, targetFrames := range [][]int{activeFrames, allFrames} {
				processAllDeltas, err := computeVmdDeltas(targetFrames, blockSize,
					sizingModel, sizingProcessMotion, sizingSet, true, all_lower_leg_bone_names, "足補正01")
				if err != nil {
					return err
				}

				prevLog := 0
				for fIndex, iFrame := range targetFrames {
					if sizingSet.IsTerminate {
						return merr.TerminateError
					}
					frame := float32(iFrame)

					// 現時点の結果
					resultAllVmdDeltas, err := computeVmdDeltas([]int{iFrame}, 1,
						sizingModel, outputMotion, sizingSet, true, leg_direction_bone_names[dIndex], "")
					if err != nil {
						return err
					}

					// ひざの位置をチェック
					resultKneeDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.KNEE.StringFromDirection(direction))
					processKneeDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.KNEE.StringFromDirection(direction))

					// 足首の位置をチェック
					resultAnkleDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))
					processAnkleDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.ANKLE.StringFromDirection(direction))

					// つま先親の位置をチェック
					resultToePDDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.TOE_P_D.StringFromDirection(direction))
					processToePDDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.TOE_P_D.StringFromDirection(direction))

					// 各関節位置がズレている場合、元の回転を焼き込む
					if resultKneeDelta.FilledGlobalPosition().Distance(processKneeDelta.FilledGlobalPosition()) > kneeThreshold ||
						resultAnkleDelta.FilledGlobalPosition().Distance(processAnkleDelta.FilledGlobalPosition()) > ankleThreshold ||
						resultToePDDelta.FilledGlobalPosition().Distance(processToePDDelta.FilledGlobalPosition()) > toeThreshold {
						for _, legBoneName := range []pmx.StandardBoneName{pmx.LEG, pmx.KNEE, pmx.ANKLE, pmx.LEG_IK} {
							boneName := legBoneName.StringFromDirection(direction)
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
							"AllCount":    fmt.Sprintf("%02d", len(targetFrames)),
							"Direction":   direction.String(),
							"FramesIndex": tIndex + 1}))
						prevLog = int(iFrame / 1000)

						incrementCompletedCount()
					}
				}
			}

			return nil
		}, nil)
	if err != nil {
		return err
	}

	if mlog.IsDebug() {
		outputVerboseMotion("足10", sizingSet.OutputMotionPath, outputMotion)
	}

	return err
}

func (su *SizingLegUsecase) checkBonesForSizingLeg(sizingSet *domain.SizingSet) (err error) {

	for _, v := range [][]interface{}{
		{sizingSet.OriginalCenterBone, pmx.CENTER.String(), true},
		{sizingSet.OriginalLowerBone, pmx.LOWER.String(), true},
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
