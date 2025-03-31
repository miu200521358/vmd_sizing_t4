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
)

// SizingLeg は、足補正処理を行います。
func SizingLeg(
	sizingSet *domain.SizingSet, moveScale *mmath.MVec3, sizingSetCount int, incrementCompletedCount func(),
) (bool, error) {
	// 対象外の場合は何もせず終了
	if !sizingSet.IsSizingLeg || sizingSet.CompletedSizingLeg {
		return false, nil
	}

	originalModel := sizingSet.OriginalConfigModel
	originalMotion := sizingSet.OriginalMotion
	sizingModel := sizingSet.SizingConfigModel
	sizingProcessMotion, err := sizingSet.OutputMotion.Copy()
	if err != nil {
		return false, err
	}

	mlog.I(mi18n.T("足補正開始", map[string]interface{}{"No": sizingSet.Index + 1}))

	// 処理対象ボーンチェック
	if checkBonesForSizingLeg(sizingSet) != nil {
		return false, err
	}

	allFrames := mmath.IntRanges(int(originalMotion.MaxFrame()) + 1)
	blockSize, _ := miter.GetBlockSize(len(allFrames) * sizingSetCount)

	// 元モデルのデフォーム結果を並列処理で取得
	originalAllDeltas, err := computeVmdDeltas(allFrames, blockSize, originalModel, originalMotion, sizingSet, true, all_gravity_lower_leg_bone_names, "足補正01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	// サイジング先モデルに対して FK 焼き込み処理
	if err := updateLegFK(sizingProcessMotion, originalAllDeltas, sizingSet); err != nil {
		return false, err
	}

	incrementCompletedCount()

	// TOE_IK キーフレームのリセット
	sizingProcessMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.TOE_IK.Left()))
	sizingProcessMotion.BoneFrames.Update(vmd.NewBoneNameFrames(pmx.TOE_IK.Right()))

	// 先モデルのデフォーム結果を並列処理で取得
	sizingAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingModel, sizingProcessMotion, sizingSet, false, gravity_bone_names, "足補正01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	// センター・グルーブ補正を実施
	centerPositions, groovePositions, err := calculateAdjustedCenter(sizingSet, allFrames, blockSize, moveScale, originalAllDeltas, sizingAllDeltas, sizingProcessMotion)
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	// センター・グルーブ位置をサイジング先モーションに反映
	updateCenter(sizingSet, allFrames, sizingProcessMotion, centerPositions, groovePositions)

	incrementCompletedCount()

	// 先モデルのデフォーム結果を並列処理で取得
	sizingOffAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingModel, sizingProcessMotion, sizingSet, false, all_lower_leg_bone_names, "足補正01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	// 足IK 補正処理
	leftLegIkPositions, leftLegIkRotations, rightLegIkPositions, rightLegIkRotations, err := calculateAdjustedLegIK(
		sizingSet, allFrames, blockSize, originalAllDeltas, sizingOffAllDeltas,
		moveScale,
	)
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	// 足IK 補正値をサイジング先モーションに反映
	updateLegIK(sizingSet, allFrames, sizingProcessMotion,
		leftLegIkPositions, leftLegIkRotations, rightLegIkPositions, rightLegIkRotations, originalAllDeltas)

	incrementCompletedCount()

	// 先モデルのデフォーム結果を並列処理で取得
	sizingAllDeltas, err = computeVmdDeltas(allFrames, blockSize, sizingModel, sizingProcessMotion, sizingSet, true, all_lower_leg_bone_names, "足補正01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	// 足FK 再計算（IK ON状態）
	leftLegRotations, leftKneeRotations, leftAnkleRotations,
		rightLegRotations, rightKneeRotations, rightAnkleRotations, err :=
		calculateAdjustedLegFK(sizingSet, allFrames, blockSize, sizingAllDeltas)
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	// 再計算した回転情報をサイジング先モーションに反映
	updateAdjustedLegFk(sizingSet, allFrames, sizingProcessMotion,
		leftLegRotations, leftKneeRotations, leftAnkleRotations,
		rightLegRotations, rightKneeRotations, rightAnkleRotations)

	incrementCompletedCount()

	// sizingSet.OutputMotion = sizingProcessMotion
	// 足補正処理の結果をサイジング先モーションに反映
	if err = updateLegResultMotion(
		sizingSet, allFrames, blockSize, sizingProcessMotion,
		incrementCompletedCount,
	); err != nil {
		return false, err
	}

	sizingSet.CompletedSizingLeg = true

	return true, nil
}

func updateLegResultMotion(
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
	for _, bone := range []*pmx.Bone{
		sizingSet.SizingCenterBone(), sizingSet.SizingGrooveBone(),
		sizingSet.SizingLeftLegIkBone(), sizingSet.SizingLeftLegBone(), sizingSet.SizingLeftKneeBone(), sizingSet.SizingLeftAnkleBone(),
		sizingSet.SizingRightLegIkBone(), sizingSet.SizingRightLegBone(), sizingSet.SizingRightKneeBone(), sizingSet.SizingRightAnkleBone(),
	} {
		outputMotion.BoneFrames.Get(bone.Name()).ForEach(func(frame float32, bf *vmd.BoneFrame) {
			processBf := sizingProcessMotion.BoneFrames.Get(bone.Name()).Get(frame)
			bf.Position = processBf.Position.Copy()
			bf.Rotation = processBf.Rotation.Copy()
			outputMotion.BoneFrames.Get(bone.Name()).Update(bf)
		})
	}

	// 中間キーフレのズレをチェック
	kneeThreshold := 0.3
	ankleThreshold := 0.2
	toeThreshold := 0.15

	err := miter.IterParallelByList([]pmx.BoneDirection{pmx.BONE_DIRECTION_LEFT, pmx.BONE_DIRECTION_RIGHT}, 1, 1,
		func(dIndex int, direction pmx.BoneDirection) error {
			for tIndex, targetFrames := range [][]int{activeFrames, allFrames} {
				processAllDeltas, err := computeVmdDeltas(targetFrames, blockSize,
					sizingModel, sizingProcessMotion, sizingSet, true, all_lower_leg_bone_names, "足補正01")
				if err != nil {
					return err
				}

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
					resultToePDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.TOE_P.StringFromDirection(direction))
					processToePDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.TOE_P.StringFromDirection(direction))

					// 各関節位置がズレている場合、元の回転を焼き込む
					if resultKneeDelta.FilledGlobalPosition().Distance(processKneeDelta.FilledGlobalPosition()) > kneeThreshold ||
						resultAnkleDelta.FilledGlobalPosition().Distance(processAnkleDelta.FilledGlobalPosition()) > ankleThreshold ||
						resultToePDelta.FilledGlobalPosition().Distance(processToePDelta.FilledGlobalPosition()) > toeThreshold {
						for _, legBoneName := range []pmx.StandardBoneName{pmx.LEG, pmx.KNEE, pmx.ANKLE, pmx.LEG_IK} {
							boneName := legBoneName.StringFromDirection(direction)
							processBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
							resultBf := outputMotion.BoneFrames.Get(boneName).Get(frame)
							if processBf.Position != nil {
								resultBf.Position = processBf.Position.Copy()
							}
							resultBf.Rotation = processBf.Rotation.Copy()
							outputMotion.InsertRegisteredBoneFrame(boneName, resultBf)
						}
					}

					if fIndex > 0 && fIndex%1000 == 0 {
						mlog.I(mi18n.T("足補正09", map[string]interface{}{
							"No":          sizingSet.Index + 1,
							"IterIndex":   fmt.Sprintf("%04d", iFrame),
							"AllCount":    fmt.Sprintf("%02d", len(targetFrames)),
							"Direction":   direction.String(),
							"FramesIndex": tIndex + 1}))
					}
				}
			}

			incrementCompletedCount()

			return nil
		}, nil)

	if mlog.IsDebug() {
		outputVerboseMotion("足08", sizingSet.OutputMotionPath, outputMotion)
	}

	return err
}

// computeInitialGravity は、対象モデルの初期重心位置を計算します。
func computeInitialGravity(sizingSet *domain.SizingSet, model *pmx.PmxModel, initialMotion *vmd.VmdMotion) *mmath.MVec3 {
	allVmdDeltas, _ := computeVmdDeltas([]int{0}, 1, model, initialMotion, sizingSet, false, gravity_bone_names, "")
	return calcGravity(allVmdDeltas[0])
}

// updateLegFK は、元モデルのデフォーム結果から FK 回転をサイジング先モーションに焼き込みます。
func updateLegFK(
	sizingProcessMotion *vmd.VmdMotion, originalAllDeltas []*delta.VmdDeltas, sizingSet *domain.SizingSet,
) error {
	for i, vmdDeltas := range originalAllDeltas {
		if sizingSet.IsTerminate {
			return merr.TerminateError
		}
		// 足（LEG）の回転補正
		for _, bone := range []*pmx.Bone{sizingSet.OriginalLeftLegBone(), sizingSet.OriginalRightLegBone()} {
			boneDelta := vmdDeltas.Bones.Get(bone.Index())
			if boneDelta == nil {
				continue
			}
			lowerDelta := vmdDeltas.Bones.Get(sizingSet.OriginalLowerBone().Index())
			sizingBf := sizingProcessMotion.BoneFrames.Get(bone.Name()).Get(boneDelta.Frame)
			sizingBf.Rotation = lowerDelta.FilledGlobalMatrix().Inverted().Muled(boneDelta.FilledGlobalMatrix()).Quaternion()
			sizingProcessMotion.InsertRegisteredBoneFrame(bone.Name(), sizingBf)
		}
		// ひざ・足首の回転補正
		for _, bone := range []*pmx.Bone{sizingSet.OriginalLeftKneeBone(), sizingSet.OriginalRightKneeBone(),
			sizingSet.OriginalLeftAnkleBone(), sizingSet.OriginalRightAnkleBone()} {
			boneDelta := vmdDeltas.Bones.Get(bone.Index())
			if boneDelta == nil {
				continue
			}
			sizingBf := sizingProcessMotion.BoneFrames.Get(bone.Name()).Get(boneDelta.Frame)
			sizingBf.Rotation = boneDelta.FilledFrameRotation()
			sizingProcessMotion.InsertRegisteredBoneFrame(bone.Name(), sizingBf)
		}

		if i > 0 && i%1000 == 0 {
			processLog("足補正02", sizingSet.Index, i, len(originalAllDeltas))
		}
	}

	if mlog.IsDebug() {
		insertIKFrames(sizingSet, sizingProcessMotion, false)
		outputVerboseMotion("足01", sizingSet.OutputMotionPath, sizingProcessMotion)
	}

	return nil
}

// insertIKFrames は、verbose モード時に IK フレームを挿入して中間結果を出力します。
func insertIKFrames(sizingSet *domain.SizingSet, sizingProcessMotion *vmd.VmdMotion, enabled bool) {
	kf := vmd.NewIkFrame(0)
	kf.Registered = true
	// 左足
	kf.IkList = append(kf.IkList, newIkEnableFrameWithBone(sizingSet.SizingLeftLegIkBone().Name(), enabled))
	kf.IkList = append(kf.IkList, newIkEnableFrameWithBone(sizingSet.SizingLeftToeIkBone().Name(), enabled))
	// 右足
	kf.IkList = append(kf.IkList, newIkEnableFrameWithBone(sizingSet.SizingRightLegIkBone().Name(), enabled))
	kf.IkList = append(kf.IkList, newIkEnableFrameWithBone(sizingSet.SizingRightToeIkBone().Name(), enabled))
	sizingProcessMotion.InsertIkFrame(kf)
}

// newIkEnableFrameWithBone は、与えられたボーン名と有効フラグから新しい IK 有効フレームを生成するヘルパー関数です。
func newIkEnableFrameWithBone(boneName string, enabled bool) *vmd.IkEnabledFrame {
	// 0 はフレーム番号の初期値として設定（必要に応じて変更してください）
	frame := vmd.NewIkEnableFrame(0)
	frame.BoneName = boneName
	frame.Enabled = enabled
	return frame
}

// calculateAdjustedCenter は、センターおよびグルーブの位置補正を並列処理で計算します。
func calculateAdjustedCenter(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, moveScale *mmath.MVec3,
	originalAllDeltas, sizingAllDeltas []*delta.VmdDeltas,
	sizingProcessMotion *vmd.VmdMotion,
) ([]*mmath.MVec3, []*mmath.MVec3, error) {
	centerPositions := make([]*mmath.MVec3, len(allFrames))
	groovePositions := make([]*mmath.MVec3, len(allFrames))

	originalGravities := make([]*mmath.MVec3, len(allFrames))
	sizingGravities := make([]*mmath.MVec3, len(allFrames))
	centerIdealPositions := make([]*mmath.MVec3, len(allFrames))

	// 元モデルと先モデルの初期重心を計算
	originalInitialGravityPos := computeInitialGravity(sizingSet, sizingSet.OriginalConfigModel, vmd.InitialMotion)
	sizingInitialGravityPos := computeInitialGravity(sizingSet, sizingSet.SizingConfigModel, vmd.InitialMotion)
	gravityRatio := sizingInitialGravityPos.Y / originalInitialGravityPos.Y

	err := miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			originalGravityPos := calcGravity(originalAllDeltas[index])
			sizingGravityPos := calcGravity(sizingAllDeltas[index])
			sizingFixCenterTargetY := originalGravityPos.Y * gravityRatio
			yDiff := sizingGravityPos.Y - sizingFixCenterTargetY

			if mlog.IsDebug() {
				originalGravities[index] = originalGravityPos
				sizingGravities[index] = sizingGravityPos

				centerIdealPositions[index] = sizingGravityPos.Copy()
				centerIdealPositions[index].Y = sizingFixCenterTargetY
			}

			frame := float32(data)
			sizingCenterBf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingCenterBone().Name()).Get(frame)
			centerPositions[index] = sizingCenterBf.Position.Muled(moveScale)

			if sizingSet.SizingGrooveVanillaBone() != nil {
				// グルーブがある場合、グルーブ位置補正を追加
				sizingGrooveBf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingGrooveVanillaBone().Name()).Get(frame)
				if sizingGrooveBf.Position == nil {
					groovePositions[index] = &mmath.MVec3{X: 0, Y: yDiff, Z: 0}
				} else {
					groovePositions[index] = sizingGrooveBf.Position.Added(&mmath.MVec3{X: 0, Y: yDiff, Z: 0})
				}
			} else {
				// グルーブがない場合、センター位置にを補正
				centerPositions[index].Add(&mmath.MVec3{X: 0, Y: yDiff, Z: 0})
			}

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正03", sizingSet.Index, iterIndex, allCount)
		})

	if mlog.IsDebug() {
		motion := vmd.NewVmdMotion("")

		for i, iFrame := range allFrames {
			frame := float32(iFrame)
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = originalGravities[i]
				motion.InsertRegisteredBoneFrame("元重心", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = sizingGravities[i]
				motion.InsertRegisteredBoneFrame("先重心", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = centerIdealPositions[i]
				motion.InsertRegisteredBoneFrame("先理想重心", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				if groovePositions[i] == nil {
					bf.Position = centerPositions[i]
				} else {
					bf.Position = centerPositions[i].Added(groovePositions[i])
				}
				motion.InsertRegisteredBoneFrame("重心センター", bf)
			}
		}

		outputVerboseMotion("足02", sizingSet.OutputMotionPath, motion)
	}

	return centerPositions, groovePositions, err
}

// updateCenter は、計算したセンター・グルーブ位置をサイジング先モーションに反映します。
func updateCenter(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	centerPositions, groovePositions []*mmath.MVec3,
) {
	for i, iFrame := range allFrames {
		frame := float32(iFrame)
		sizingCenterBf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingCenterBone().Name()).Get(frame)
		sizingCenterBf.Position = centerPositions[i]
		sizingProcessMotion.InsertRegisteredBoneFrame(sizingSet.SizingCenterBone().Name(), sizingCenterBf)

		if sizingSet.SizingGrooveVanillaBone() != nil {
			sizingGrooveBf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingGrooveBone().Name()).Get(frame)
			sizingGrooveBf.Position = groovePositions[i]
			sizingProcessMotion.InsertRegisteredBoneFrame(sizingSet.SizingGrooveBone().Name(), sizingGrooveBf)
		}

		if i > 0 && i%1000 == 0 {
			processLog("足補正04", sizingSet.Index, i, len(allFrames))
		}
	}

	if mlog.IsDebug() {
		outputVerboseMotion("足03", sizingSet.OutputMotionPath, sizingProcessMotion)
	}
}

// calculateAdjustedLegIK は、足IK 補正の計算を並列処理で行い、各フレームごとの位置・回転補正値を算出します。
func calculateAdjustedLegIK(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int,
	originalAllDeltas, sizingOffAllDeltas []*delta.VmdDeltas,
	moveScale *mmath.MVec3,
) (
	leftLegIkPositions []*mmath.MVec3, leftLegIkRotations []*mmath.MQuaternion,
	rightLegIkPositions []*mmath.MVec3, rightLegIkRotations []*mmath.MQuaternion, err error,
) {
	leftLegIkPositions = make([]*mmath.MVec3, len(allFrames))
	leftLegIkRotations = make([]*mmath.MQuaternion, len(allFrames))
	rightLegIkPositions = make([]*mmath.MVec3, len(allFrames))
	rightLegIkRotations = make([]*mmath.MQuaternion, len(allFrames))

	leftLegBeforePositions := make([]*mmath.MVec3, len(allFrames))
	rightLegBeforePositions := make([]*mmath.MVec3, len(allFrames))
	leftToeBeforePositions := make([]*mmath.MVec3, len(allFrames))
	rightToeBeforePositions := make([]*mmath.MVec3, len(allFrames))
	leftHeelBeforePositions := make([]*mmath.MVec3, len(allFrames))
	rightHeelBeforePositions := make([]*mmath.MVec3, len(allFrames))
	leftToeIdealPositions := make([]*mmath.MVec3, len(allFrames))
	rightToeIdealPositions := make([]*mmath.MVec3, len(allFrames))
	leftHeelIdealPositions := make([]*mmath.MVec3, len(allFrames))
	rightHeelIdealPositions := make([]*mmath.MVec3, len(allFrames))
	leftLegAfterPositions := make([]*mmath.MVec3, len(allFrames))
	rightLegAfterPositions := make([]*mmath.MVec3, len(allFrames))

	originalLeftDiff := sizingSet.OriginalLeftToeTailBone().Position.Subed(sizingSet.OriginalLeftLegIkBone().Position)
	sizingLeftDiff := sizingSet.SizingLeftToeTailBone().Position.Subed(sizingSet.SizingLeftLegIkBone().Position)
	leftAnkleScale := sizingLeftDiff.Dived(originalLeftDiff).Absed()
	leftAnkleScale.X = 1.0

	originalRightDiff := sizingSet.OriginalRightToeTailBone().Position.Subed(sizingSet.OriginalRightLegIkBone().Position)
	sizingRightDiff := sizingSet.SizingRightToeTailBone().Position.Subed(sizingSet.SizingRightLegIkBone().Position)
	rightAnkleScale := sizingRightDiff.Dived(originalRightDiff).Absed()
	rightAnkleScale.X = 1.0

	leftLegIkDirectionSlope := sizingSet.SizingLeftToeTailBone().Position.Subed(sizingSet.SizingLeftAnkleBone().Position).Normalize()
	leftLegIkUpSlope := sizingSet.SizingLeftToeTailBone().Position.Subed(sizingSet.SizingLeftToePBone().Position).Normalize()
	leftLegIkCrossSlope := leftLegIkUpSlope.Cross(leftLegIkDirectionSlope).Normalize()
	leftLegIkSlopeQuat := mmath.NewMQuaternionFromDirection(leftLegIkDirectionSlope, leftLegIkCrossSlope)

	rightLegIkDirectionSlope := sizingSet.SizingRightToeTailBone().Position.Subed(sizingSet.SizingRightAnkleBone().Position).Normalize()
	rightLegIkUpSlope := sizingSet.SizingRightToeTailBone().Position.Subed(sizingSet.SizingRightToePBone().Position).Normalize()
	rightLegIkCrossSlope := rightLegIkUpSlope.Cross(rightLegIkDirectionSlope).Normalize()
	rightLegIkSlopeQuat := mmath.NewMQuaternionFromDirection(rightLegIkDirectionSlope, rightLegIkCrossSlope)

	sizingLeftLegIkParentBone, _ := sizingSet.SizingConfigModel.Bones.Get(sizingSet.SizingLeftLegIkBone().ParentIndex)
	sizingRightLegIkParentBone, _ := sizingSet.SizingConfigModel.Bones.Get(sizingSet.SizingRightLegIkBone().ParentIndex)

	leftLegIkDiff := sizingSet.SizingLeftLegIkBone().Position.Subed(sizingLeftLegIkParentBone.Position)
	rightLegIkDiff := sizingSet.SizingRightLegIkBone().Position.Subed(sizingRightLegIkParentBone.Position)

	err = miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			// 元モデルから各種足ボーンの位置取得
			originalLeftHeelDelta := originalAllDeltas[index].Bones.Get(sizingSet.OriginalLeftHeelBone().Index())
			originalLeftToeTailDelta := originalAllDeltas[index].Bones.Get(sizingSet.OriginalLeftToeTailBone().Index())
			originalRightHeelDelta := originalAllDeltas[index].Bones.Get(sizingSet.OriginalRightHeelBone().Index())
			originalRightToeTailDelta := originalAllDeltas[index].Bones.Get(sizingSet.OriginalRightToeTailBone().Index())

			// サイジング先モデルの各足ボーンのデフォーム結果取得
			sizingLeftLegIkParentDelta := sizingOffAllDeltas[index].Bones.Get(sizingLeftLegIkParentBone.Index())
			sizingLeftAnkleDelta := sizingOffAllDeltas[index].Bones.Get(sizingSet.SizingLeftAnkleBone().Index())
			sizingLeftHeelDelta := sizingOffAllDeltas[index].Bones.Get(sizingSet.SizingLeftHeelBone().Index())
			sizingLeftToePDelta := sizingOffAllDeltas[index].Bones.Get(sizingSet.SizingLeftToePBone().Index())
			sizingLeftToeTailDelta := sizingOffAllDeltas[index].Bones.Get(sizingSet.SizingLeftToeTailBone().Index())
			sizingRightLegIkParentDelta := sizingOffAllDeltas[index].Bones.Get(sizingRightLegIkParentBone.Index())
			sizingRightAnkleDelta := sizingOffAllDeltas[index].Bones.Get(sizingSet.SizingRightAnkleBone().Index())
			sizingRightHeelDelta := sizingOffAllDeltas[index].Bones.Get(sizingSet.SizingRightHeelBone().Index())
			sizingRightToePDelta := sizingOffAllDeltas[index].Bones.Get(sizingSet.SizingRightToePBone().Index())
			sizingRightToeTailDelta := sizingOffAllDeltas[index].Bones.Get(sizingSet.SizingRightToeTailBone().Index())

			// 足IK の初期位置計算 （足首位置から各IKボーンの親へのオフセット）
			leftLegIkPositions[index] = sizingLeftLegIkParentDelta.FilledGlobalMatrix().Inverted().MulVec3(
				sizingLeftAnkleDelta.FilledGlobalPosition().Subed(leftLegIkDiff))
			rightLegIkPositions[index] = sizingRightLegIkParentDelta.FilledGlobalMatrix().Inverted().MulVec3(
				sizingRightAnkleDelta.FilledGlobalPosition().Subed(rightLegIkDiff))

			// 足IK の回転
			leftLegFkDirectionSlope := sizingLeftToeTailDelta.FilledGlobalPosition().Subed(
				sizingLeftAnkleDelta.FilledGlobalPosition()).Normalize()
			leftLegFkUpSlope := sizingLeftToeTailDelta.FilledGlobalPosition().Subed(
				sizingLeftToePDelta.FilledGlobalPosition()).Normalize()
			leftLegCrossSlope := leftLegFkUpSlope.Cross(leftLegFkDirectionSlope).Normalize()
			leftLegFkSlopeQuat := mmath.NewMQuaternionFromDirection(leftLegFkDirectionSlope, leftLegCrossSlope)
			leftLegIkRotations[index] = leftLegFkSlopeQuat.Muled(leftLegIkSlopeQuat.Inverted())

			rightLegFkDirectionSlope := sizingRightToeTailDelta.FilledGlobalPosition().Subed(
				sizingRightAnkleDelta.FilledGlobalPosition()).Normalize()
			rightLegFkUpSlope := sizingRightToeTailDelta.FilledGlobalPosition().Subed(
				sizingRightToePDelta.FilledGlobalPosition()).Normalize()
			rightLegCrossSlope := rightLegFkUpSlope.Cross(rightLegFkDirectionSlope).Normalize()
			rightLegFkSlopeQuat := mmath.NewMQuaternionFromDirection(rightLegFkDirectionSlope, rightLegCrossSlope)
			rightLegIkRotations[index] = rightLegFkSlopeQuat.Muled(rightLegIkSlopeQuat.Inverted())

			if mlog.IsDebug() {
				leftLegBeforePositions[index] = sizingLeftAnkleDelta.FilledGlobalPosition().Copy()
				rightLegBeforePositions[index] = sizingRightAnkleDelta.FilledGlobalPosition().Copy()
				leftLegAfterPositions[index] = leftLegIkPositions[index].Copy()
				rightLegAfterPositions[index] = rightLegIkPositions[index].Copy()
			}

			// 足IK の Y 軸補正
			calcLegIkPositionY(index, leftLegIkPositions,
				sizingSet.OriginalLeftAnkleBone(),
				originalLeftToeTailDelta, originalLeftHeelDelta,
				sizingLeftToeTailDelta, sizingLeftHeelDelta, moveScale,
				leftToeBeforePositions, leftToeIdealPositions, leftHeelBeforePositions, leftHeelIdealPositions)
			calcLegIkPositionY(index, rightLegIkPositions,
				sizingSet.OriginalRightAnkleBone(),
				originalRightToeTailDelta, originalRightHeelDelta,
				sizingRightToeTailDelta, sizingRightHeelDelta, moveScale,
				rightToeBeforePositions, rightToeIdealPositions, rightHeelBeforePositions, rightHeelIdealPositions)

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正05", sizingSet.Index, iterIndex, allCount)
		})

	if mlog.IsDebug() {
		motion := vmd.NewVmdMotion("")

		for i, iFrame := range allFrames {
			frame := float32(iFrame)
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = leftLegBeforePositions[i]
				motion.InsertRegisteredBoneFrame("左足Y補正前", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = rightLegBeforePositions[i]
				motion.InsertRegisteredBoneFrame("右足Y補正前", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = leftLegAfterPositions[i]
				motion.InsertRegisteredBoneFrame("左足Y補正後", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = rightLegAfterPositions[i]
				motion.InsertRegisteredBoneFrame("右足Y補正後", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = leftToeBeforePositions[i]
				motion.InsertRegisteredBoneFrame("左つま先補正前", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = leftToeIdealPositions[i]
				motion.InsertRegisteredBoneFrame("左つま先理想", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = rightToeBeforePositions[i]
				motion.InsertRegisteredBoneFrame("右つま先補正前", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = rightToeIdealPositions[i]
				motion.InsertRegisteredBoneFrame("右つま先理想", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = leftHeelBeforePositions[i]
				motion.InsertRegisteredBoneFrame("左かかと補正前", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = leftHeelIdealPositions[i]
				motion.InsertRegisteredBoneFrame("左かかと理想", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = rightHeelBeforePositions[i]
				motion.InsertRegisteredBoneFrame("右かかと補正前", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = rightHeelIdealPositions[i]
				motion.InsertRegisteredBoneFrame("右かかと理想", bf)
			}
		}

		outputVerboseMotion("足04", sizingSet.OutputMotionPath, motion)
	}

	return
}

// updateLegIK は、計算済みの足IK 補正位置と回転値をモーションデータに反映させます。
func updateLegIK(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	leftLegIkPositions []*mmath.MVec3, leftLegIkRotations []*mmath.MQuaternion,
	rightLegIkPositions []*mmath.MVec3, rightLegIkRotations []*mmath.MQuaternion,
	originalAllDeltas []*delta.VmdDeltas,
) {
	inheritanceLeftXPositions := make([]*mmath.MVec3, len(allFrames))
	inheritanceLeftYPositions := make([]*mmath.MVec3, len(allFrames))
	inheritanceLeftZPositions := make([]*mmath.MVec3, len(allFrames))
	inheritanceRightXPositions := make([]*mmath.MVec3, len(allFrames))
	inheritanceRightYPositions := make([]*mmath.MVec3, len(allFrames))
	inheritanceRightZPositions := make([]*mmath.MVec3, len(allFrames))

	for i, iFrame := range allFrames {
		if i > 0 {
			// 前フレームと同じ足首位置なら前フレームの値を継承
			originalLeftAnklePosition := originalAllDeltas[i].Bones.Get(sizingSet.OriginalLeftAnkleBone().Index()).FilledGlobalPosition()
			originalRightAnklePosition := originalAllDeltas[i].Bones.Get(sizingSet.OriginalRightAnkleBone().Index()).FilledGlobalPosition()
			originalLeftAnklePrevPosition := originalAllDeltas[i-1].Bones.Get(sizingSet.OriginalLeftAnkleBone().Index()).FilledGlobalPosition()
			originalRightAnklePrevPosition := originalAllDeltas[i-1].Bones.Get(sizingSet.OriginalRightAnkleBone().Index()).FilledGlobalPosition()

			if mmath.NearEquals(originalLeftAnklePrevPosition.X, originalLeftAnklePosition.X, 1e-2) {
				leftLegIkPositions[i].X = leftLegIkPositions[i-1].X
				if mlog.IsDebug() {
					inheritanceLeftXPositions[i] = &mmath.MVec3{X: leftLegIkPositions[i].X, Y: 0, Z: 0}
				}
			}
			if mmath.NearEquals(originalLeftAnklePrevPosition.Y, originalLeftAnklePosition.Y, 1e-2) {
				leftLegIkPositions[i].Y = leftLegIkPositions[i-1].Y
				if mlog.IsDebug() {
					inheritanceLeftYPositions[i] = &mmath.MVec3{X: 0, Y: leftLegIkPositions[i].Y, Z: 0}
				}
			}
			if mmath.NearEquals(originalLeftAnklePrevPosition.Z, originalLeftAnklePosition.Z, 1e-2) {
				leftLegIkPositions[i].Z = leftLegIkPositions[i-1].Z
				if mlog.IsDebug() {
					inheritanceLeftZPositions[i] = &mmath.MVec3{X: 0, Y: 0, Z: leftLegIkPositions[i].Z}
				}
			}
			if mmath.NearEquals(originalRightAnklePrevPosition.X, originalRightAnklePosition.X, 1e-2) {
				rightLegIkPositions[i].X = rightLegIkPositions[i-1].X
				if mlog.IsDebug() {
					inheritanceRightXPositions[i] = &mmath.MVec3{X: rightLegIkPositions[i].X, Y: 0, Z: 0}
				}
			}
			if mmath.NearEquals(originalRightAnklePrevPosition.Y, originalRightAnklePosition.Y, 1e-2) {
				rightLegIkPositions[i].Y = rightLegIkPositions[i-1].Y
				if mlog.IsDebug() {
					inheritanceRightYPositions[i] = &mmath.MVec3{X: 0, Y: rightLegIkPositions[i].Y, Z: 0}
				}
			}
			if mmath.NearEquals(originalRightAnklePrevPosition.Z, originalRightAnklePosition.Z, 1e-2) {
				rightLegIkPositions[i].Z = rightLegIkPositions[i-1].Z
				if mlog.IsDebug() {
					inheritanceRightZPositions[i] = &mmath.MVec3{X: 0, Y: 0, Z: rightLegIkPositions[i].Z}
				}
			}
		}
		frame := float32(iFrame)
		// 各フレームの IK 補正値を更新
		rightLegIkBf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingRightLegIkBone().Name()).Get(frame)
		rightLegIkBf.Position = rightLegIkPositions[i]
		rightLegIkBf.Rotation = rightLegIkRotations[i]
		sizingProcessMotion.InsertRegisteredBoneFrame(sizingSet.SizingRightLegIkBone().Name(), rightLegIkBf)

		leftLegIkBf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingLeftLegIkBone().Name()).Get(frame)
		leftLegIkBf.Position = leftLegIkPositions[i]
		leftLegIkBf.Rotation = leftLegIkRotations[i]
		sizingProcessMotion.InsertRegisteredBoneFrame(sizingSet.SizingLeftLegIkBone().Name(), leftLegIkBf)

		if i > 0 && i%1000 == 0 {
			processLog("足補正06", sizingSet.Index, i, len(allFrames))
		}
	}

	if mlog.IsDebug() {
		outputVerboseMotion("足05", sizingSet.OutputMotionPath, sizingProcessMotion)
	}

	if mlog.IsDebug() {
		motion := vmd.NewVmdMotion("")

		for i, iFrame := range allFrames {
			frame := float32(iFrame)
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = inheritanceLeftXPositions[i]
				motion.InsertRegisteredBoneFrame("左足継承X", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = inheritanceLeftYPositions[i]
				motion.InsertRegisteredBoneFrame("左足継承Y", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = inheritanceLeftZPositions[i]
				motion.InsertRegisteredBoneFrame("左足継承Z", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = inheritanceRightXPositions[i]
				motion.InsertRegisteredBoneFrame("右足継承X", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = inheritanceRightYPositions[i]
				motion.InsertRegisteredBoneFrame("右足継承Y", bf)
			}
			{
				bf := vmd.NewBoneFrame(frame)
				bf.Position = inheritanceRightZPositions[i]
				motion.InsertRegisteredBoneFrame("右足継承Z", bf)
			}
		}

		outputVerboseMotion("足06", sizingSet.OutputMotionPath, motion)
	}
}

// calculateAdjustedLegFK は、IK ON状態で足FK の回転を再計算し、その結果を配列で返します。
func calculateAdjustedLegFK(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, sizingAllDeltas []*delta.VmdDeltas,

) ([]*mmath.MQuaternion, []*mmath.MQuaternion, []*mmath.MQuaternion,
	[]*mmath.MQuaternion, []*mmath.MQuaternion, []*mmath.MQuaternion, error) {
	leftLegRotations := make([]*mmath.MQuaternion, len(allFrames))
	leftKneeRotations := make([]*mmath.MQuaternion, len(allFrames))
	leftAnkleRotations := make([]*mmath.MQuaternion, len(allFrames))
	rightLegRotations := make([]*mmath.MQuaternion, len(allFrames))
	rightKneeRotations := make([]*mmath.MQuaternion, len(allFrames))
	rightAnkleRotations := make([]*mmath.MQuaternion, len(allFrames))

	err := miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			leftLegRotations[index] = sizingAllDeltas[index].Bones.Get(sizingSet.SizingLeftLegBone().Index()).FilledFrameRotation()
			leftKneeRotations[index] = sizingAllDeltas[index].Bones.Get(sizingSet.SizingLeftKneeBone().Index()).FilledFrameRotation()
			leftAnkleRotations[index] = sizingAllDeltas[index].Bones.Get(sizingSet.SizingLeftAnkleBone().Index()).FilledFrameRotation()

			rightLegRotations[index] = sizingAllDeltas[index].Bones.Get(sizingSet.SizingRightLegBone().Index()).FilledFrameRotation()
			rightKneeRotations[index] = sizingAllDeltas[index].Bones.Get(sizingSet.SizingRightKneeBone().Index()).FilledFrameRotation()
			rightAnkleRotations[index] = sizingAllDeltas[index].Bones.Get(sizingSet.SizingRightAnkleBone().Index()).FilledFrameRotation()
			return nil
		},
		func(iterIndex, allCount int) {
			processLog("足補正07", sizingSet.Index, iterIndex, allCount)
		})

	return leftLegRotations, leftKneeRotations, leftAnkleRotations, rightLegRotations, rightKneeRotations, rightAnkleRotations, err
}

func calcLegIkPositionY(
	index int, legIkPositions []*mmath.MVec3, originalAnkleBone *pmx.Bone,
	originalToeTailDelta, originalHeelDelta,
	sizingToeTailDelta, sizingHeelDelta *delta.BoneDelta, moveScale *mmath.MVec3,
	toeBeforePositions, toeIdealPositions, heelBeforePositions, heelIdealPositions []*mmath.MVec3,
) {
	if originalToeTailDelta.FilledGlobalPosition().Y <= originalHeelDelta.FilledGlobalPosition().Y ||
		mmath.NearEquals(originalToeTailDelta.FilledGlobalPosition().Y, originalHeelDelta.FilledGlobalPosition().Y, 1e-1) {
		// つま先の方がかかとより低い場合
		originalToeTailY := originalToeTailDelta.FilledGlobalPosition().Y

		// つま先のY座標を元モデルのつま先のY座標*スケールに合わせる
		idealSizingToeTailY := originalToeTailY * moveScale.Y

		// 現時点のつま先のY座標(足IKの回転結果を適用させて求め直す)
		actualToeTailY := sizingToeTailDelta.FilledGlobalPosition().Y

		if mlog.IsDebug() {
			toeBeforePositions[index] = sizingToeTailDelta.FilledGlobalPosition().Copy()
			toeIdealPositions[index] = sizingToeTailDelta.FilledGlobalPosition().Copy()
			toeIdealPositions[index].Y = idealSizingToeTailY
		}

		toeDiff := max(0, idealSizingToeTailY) - actualToeTailY
		// toeDiff += originalAnkleBone.Position.Y - sizingAnkleBone.Position.Y
		lerpToeDiff := mmath.Lerp(toeDiff, 0,
			max(0, originalToeTailDelta.FilledGlobalPosition().Y/originalAnkleBone.Position.Y))
		// 足首Y位置に近付くにつれて補正を弱める
		legIkPositions[index].Y += lerpToeDiff
		// ankleDiff := sizingLeftToeTailY - originalLeftToeTailY
		// legIkPositions[index].Y += ankleDiff

		return
	}

	// かかとの方がつま先より低い場合
	originalHeelY := originalHeelDelta.FilledGlobalPosition().Y

	// かかとのY座標を元モデルのかかとのY座標*スケールに合わせる
	idealSizingHeelY := originalHeelY * moveScale.Y

	// 現時点のかかとのY座標
	actualSizingHeelY := sizingHeelDelta.FilledGlobalPosition().Y

	if mlog.IsDebug() {
		heelBeforePositions[index] = sizingHeelDelta.FilledGlobalPosition().Copy()
		heelIdealPositions[index] = sizingHeelDelta.FilledGlobalPosition().Copy()
		heelIdealPositions[index].Y = idealSizingHeelY
	}

	heelDiff := idealSizingHeelY - actualSizingHeelY
	// heelDiff += originalAnkleBone.Position.Y - sizingAnkleBone.Position.Y
	lerpHeelDiff := mmath.Lerp(heelDiff, 0,
		originalHeelDelta.FilledGlobalPosition().Y/originalAnkleBone.Position.Y)
	// 足首Y位置に近付くにつれて補正を弱める
	legIkPositions[index].Y += lerpHeelDiff

	// ankleDiff := (sizingLeftHeelY - sizingLeftAnkleY) - (originalLeftHeelY - originalLeftAnkleY)
	// legIkPositions[index].Y += ankleDiff

	return
}

func updateAdjustedLegFk(
	sizingSet *domain.SizingSet, allFrames []int,
	sizingProcessMotion *vmd.VmdMotion,
	leftLegRotations, leftKneeRotations, leftAnkleRotations,
	rightLegRotations, rightKneeRotations, rightAnkleRotations []*mmath.MQuaternion,
) {

	// サイジング先にFKを焼き込み
	for i, iFrame := range allFrames {
		if sizingSet.IsTerminate {
			return
		}

		frame := float32(iFrame)

		{
			bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingLeftLegBone().Name()).Get(frame)
			bf.Rotation = leftLegRotations[i]
			sizingProcessMotion.InsertRegisteredBoneFrame(sizingSet.SizingLeftLegBone().Name(), bf)
		}
		{
			bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingLeftKneeBone().Name()).Get(frame)
			bf.Rotation = leftKneeRotations[i]
			sizingProcessMotion.InsertRegisteredBoneFrame(sizingSet.SizingLeftKneeBone().Name(), bf)
		}
		{
			bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingLeftAnkleBone().Name()).Get(frame)
			bf.Rotation = leftAnkleRotations[i]
			sizingProcessMotion.InsertRegisteredBoneFrame(sizingSet.SizingLeftAnkleBone().Name(), bf)
		}
		{
			bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingRightLegBone().Name()).Get(frame)
			bf.Rotation = rightLegRotations[i]
			sizingProcessMotion.InsertRegisteredBoneFrame(sizingSet.SizingRightLegBone().Name(), bf)
		}
		{
			bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingRightKneeBone().Name()).Get(frame)
			bf.Rotation = rightKneeRotations[i]
			sizingProcessMotion.InsertRegisteredBoneFrame(sizingSet.SizingRightKneeBone().Name(), bf)
		}
		{
			bf := sizingProcessMotion.BoneFrames.Get(sizingSet.SizingRightAnkleBone().Name()).Get(frame)
			bf.Rotation = rightAnkleRotations[i]
			sizingProcessMotion.InsertRegisteredBoneFrame(sizingSet.SizingRightAnkleBone().Name(), bf)
		}

		if i > 0 && i%1000 == 0 {
			processLog("足補正08", sizingSet.Index, i, len(allFrames))
		}
	}

	if mlog.IsDebug() {
		outputVerboseMotion("足07", sizingSet.OutputMotionPath, sizingProcessMotion)
	}
}

func checkBonesForSizingLeg(sizingSet *domain.SizingSet) (err error) {

	for _, v := range [][]interface{}{
		{sizingSet.OriginalCenterBone, pmx.CENTER.String()},
		{sizingSet.OriginalLowerBone, pmx.LOWER.String()},
		{sizingSet.OriginalLeftLegIkBone, pmx.LEG_IK.Left()},
		{sizingSet.OriginalLeftLegBone, pmx.LEG.Left()},
		{sizingSet.OriginalLeftKneeBone, pmx.KNEE.Left()},
		{sizingSet.OriginalLeftAnkleBone, pmx.ANKLE.Left()},
		{sizingSet.OriginalLeftToeIkBone, pmx.TOE_IK.Left()},
		{sizingSet.OriginalLeftToeTailBone, pmx.TOE_T.Left()},
		{sizingSet.OriginalLeftHeelBone, pmx.HEEL.Left()},
		{sizingSet.OriginalLeftToePBone, pmx.TOE_P.Left()},
		{sizingSet.OriginalRightLegIkBone, pmx.LEG_IK.Right()},
		{sizingSet.OriginalRightLegBone, pmx.LEG.Right()},
		{sizingSet.OriginalRightKneeBone, pmx.KNEE.Right()},
		{sizingSet.OriginalRightAnkleBone, pmx.ANKLE.Right()},
		{sizingSet.OriginalRightToeIkBone, pmx.TOE_IK.Right()},
		{sizingSet.OriginalRightToeTailBone, pmx.TOE_T.Right()},
		{sizingSet.OriginalRightHeelBone, pmx.HEEL.Right()},
		{sizingSet.OriginalRightToePBone, pmx.TOE_P.Right()},
	} {
		getFunc := v[0].(func() *pmx.Bone)
		boneName := v[1].(string)

		if getFunc() == nil {
			mlog.WT(mi18n.T("ボーン不足"), mi18n.T("ボーン不足エラー", map[string]interface{}{
				"Process": mi18n.T("足補正"), "No": sizingSet.Index + 1, "ModelType": "元モデル", "BoneName": boneName}))
			err = merr.NameNotFoundError
		}
	}

	// ------------------------------------------

	// グルーブはサイジング先に元々存在している場合のみ取得
	sizingSet.SizingGrooveVanillaBone()

	for _, v := range [][]interface{}{
		{sizingSet.SizingCenterBone, pmx.CENTER.String()},
		{sizingSet.SizingLowerBone, pmx.LOWER.String()},
		{sizingSet.SizingLeftLegIkBone, pmx.LEG_IK.Left()},
		{sizingSet.SizingLeftLegBone, pmx.LEG.Left()},
		{sizingSet.SizingLeftKneeBone, pmx.KNEE.Left()},
		{sizingSet.SizingLeftAnkleBone, pmx.ANKLE.Left()},
		{sizingSet.SizingLeftToeIkBone, pmx.TOE_IK.Left()},
		{sizingSet.SizingLeftToeTailBone, pmx.TOE_T.Left()},
		{sizingSet.SizingLeftHeelBone, pmx.HEEL.Left()},
		{sizingSet.SizingLeftToePBone, pmx.TOE_P.Left()},
		{sizingSet.SizingRightLegIkBone, pmx.LEG_IK.Right()},
		{sizingSet.SizingRightLegBone, pmx.LEG.Right()},
		{sizingSet.SizingRightKneeBone, pmx.KNEE.Right()},
		{sizingSet.SizingRightAnkleBone, pmx.ANKLE.Right()},
		{sizingSet.SizingRightToeIkBone, pmx.TOE_IK.Right()},
		{sizingSet.SizingRightToeTailBone, pmx.TOE_T.Right()},
		{sizingSet.SizingRightHeelBone, pmx.HEEL.Right()},
		{sizingSet.SizingRightToePBone, pmx.TOE_P.Right()},
	} {
		getFunc := v[0].(func() *pmx.Bone)
		boneName := v[1].(string)

		if getFunc() == nil {
			mlog.WT(mi18n.T("ボーン不足"), mi18n.T("ボーン不足エラー", map[string]interface{}{
				"Process": mi18n.T("足補正"), "No": sizingSet.Index + 1, "ModelType": "先モデル", "BoneName": boneName}))
			err = merr.NameNotFoundError
		}
	}

	return err
}

func calcGravity(vmdDeltas *delta.VmdDeltas) *mmath.MVec3 {
	gravityPos := mmath.NewMVec3()

	for _, boneName := range gravity_bone_names {
		fromBoneDelta := vmdDeltas.Bones.GetByName(boneName)
		if fromBoneDelta == nil || fromBoneDelta.Bone == nil {
			continue
		}
		gravityBoneNames := fromBoneDelta.Bone.Config().CenterOfGravityBoneNames
		if len(gravityBoneNames) == 0 {
			continue
		}
		toBoneName := gravityBoneNames[0].StringFromDirection(fromBoneDelta.Bone.Direction())
		toBoneDelta := vmdDeltas.Bones.GetByName(toBoneName)
		if toBoneDelta == nil || toBoneDelta.Bone == nil {
			continue
		}
		gravity := fromBoneDelta.Bone.Config().CenterOfGravity

		gravityPos.Add(toBoneDelta.FilledGlobalPosition().Added(
			fromBoneDelta.FilledGlobalPosition()).MuledScalar(0.5 * gravity))
	}

	return gravityPos
}
