package usecase

import (
	"fmt"
	"math"
	"miu200521358/vmd_sizing_t4/pkg/domain"

	"github.com/miu200521358/mlib_go/pkg/config/merr"
	"github.com/miu200521358/mlib_go/pkg/config/mi18n"
	"github.com/miu200521358/mlib_go/pkg/config/mlog"
	"github.com/miu200521358/mlib_go/pkg/domain/delta"
	"github.com/miu200521358/mlib_go/pkg/domain/mmath"
	"github.com/miu200521358/mlib_go/pkg/domain/pmx"
	"github.com/miu200521358/mlib_go/pkg/domain/vmd"
	"github.com/miu200521358/mlib_go/pkg/infrastructure/mfile"
	"github.com/miu200521358/mlib_go/pkg/infrastructure/miter"
	"github.com/miu200521358/mlib_go/pkg/infrastructure/repository"
	"github.com/miu200521358/mlib_go/pkg/usecase/deform"
)

// SizingWrist は手首位置合わせ処理を行います。
func SizingWrist(sizingSet *domain.SizingSet, sizingSetCount int, incrementCompletedCount func()) (bool, error) {
	// 対象外の場合は何もせず終了
	if !sizingSet.IsSizingWrist || sizingSet.CompletedSizingWrist {
		return false, nil
	}

	originalMotion := sizingSet.OriginalMotion
	sizingProcessMotion, err := sizingSet.OutputMotion.Copy()
	if err != nil {
		return false, err
	}

	mlog.I(mi18n.T("手首位置合わせ開始", map[string]interface{}{"No": sizingSet.Index + 1}))

	// 処理対象ボーンチェック
	if err := checkBonesForSizingWrist(sizingSet); err != nil {
		return false, err
	}

	allFrames := mmath.IntRanges(int(originalMotion.MaxFrame()) + 1)
	blockSize, _ := miter.GetBlockSize(len(allFrames) * sizingSetCount)

	// 元モデルのデフォーム結果を並列処理で取得
	originalAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.OriginalConfigModel, originalMotion, sizingSet, true, append(all_arm_bone_names[0], all_arm_bone_names[1]...), "手首位置合わせ01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	sizingAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel, sizingProcessMotion, sizingSet, true, append(all_arm_bone_names[0], all_arm_bone_names[1]...), "手首位置合わせ01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	if err := calculateAdjustedWrist(sizingSet, allFrames, blockSize, sizingAllDeltas, originalAllDeltas, sizingProcessMotion, incrementCompletedCount); err != nil {
		return false, err
	}

	incrementCompletedCount()

	if err := updateWristResultMotion(sizingSet, allFrames, blockSize, sizingProcessMotion, incrementCompletedCount); err != nil {
		return false, err
	}

	incrementCompletedCount()

	sizingSet.CompletedSizingWrist = true

	return true, nil
}

func calculateAdjustedWrist(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int,
	sizingAllDeltas, originalAllDeltas []*delta.VmdDeltas, sizingProcessMotion *vmd.VmdMotion,
	incrementCompletedCount func(),
) error {
	if mlog.IsDebug() {
		outputVerboseMotion("位置01", sizingSet.OutputMotionPath, sizingProcessMotion)
	}

	armIkBones := make([]*pmx.Bone, 2)
	for i, direction := range directions {
		armIkBones[i] = createArmIkBone(sizingSet, direction)
	}

	if mlog.IsDebug() {
		ikModel := pmx.NewPmxModel("")
		ikModel.Bones = pmx.NewBones(0)
		sizingSet.SizingConfigModel.Bones.ForEach(func(index int, b *pmx.Bone) bool {
			ikModel.Bones.Append(b.Copy().(*pmx.Bone))
			return true
		})
		sizingSet.SizingConfigModel.DisplaySlots.ForEach(func(index int, d *pmx.DisplaySlot) bool {
			ikModel.DisplaySlots.Append(d.Copy().(*pmx.DisplaySlot))
			return true
		})

		for _, ikBone := range armIkBones {
			ikBone.BoneFlag = pmx.BONE_FLAG_CAN_TRANSLATE | pmx.BONE_FLAG_CAN_ROTATE | pmx.BONE_FLAG_CAN_TRANSLATE | pmx.BONE_FLAG_IS_VISIBLE | pmx.BONE_FLAG_IS_IK
			ikModel.Bones.Append(ikBone)

			slot := pmx.NewDisplaySlot()
			slot.SetName(ikBone.Name())
			slot.References = append(slot.References, pmx.NewDisplaySlotReferenceByValues(pmx.DISPLAY_TYPE_BONE, ikBone.Index()))
			slot.SpecialFlag = pmx.SPECIAL_FLAG_OFF
			ikModel.DisplaySlots.Append(slot)
		}

		rep := repository.NewPmxRepository(false)
		outputPath := mfile.CreateOutputPath(sizingSet.SizingConfigModel.Path(), "IK")
		rep.Save(outputPath, ikModel, true)
	}

	originalWristPositions := make([][]*mmath.MVec3, 2)

	sizingWristPositions := make([][]*mmath.MVec3, 2)
	sizingWristFromNeckIdealPositions := make([][]*mmath.MVec3, 2)
	sizingWristFromTrunkIdealPositions := make([][]*mmath.MVec3, 2)
	sizingWristIdealPositions := make([][]*mmath.MVec3, 2)

	sizingArmRotations := make([][]*mmath.MQuaternion, 2)
	sizingElbowRotations := make([][]*mmath.MQuaternion, 2)
	sizingWristRotations := make([][]*mmath.MQuaternion, 2)

	for i := range directions {
		originalWristPositions[i] = make([]*mmath.MVec3, len(allFrames))

		sizingWristPositions[i] = make([]*mmath.MVec3, len(allFrames))
		sizingWristFromNeckIdealPositions[i] = make([]*mmath.MVec3, len(allFrames))
		sizingWristFromTrunkIdealPositions[i] = make([]*mmath.MVec3, len(allFrames))
		sizingWristIdealPositions[i] = make([]*mmath.MVec3, len(allFrames))

		sizingArmRotations[i] = make([]*mmath.MQuaternion, len(allFrames))
		sizingElbowRotations[i] = make([]*mmath.MQuaternion, len(allFrames))
		sizingWristRotations[i] = make([]*mmath.MQuaternion, len(allFrames))
	}

	originalNeckRootToTrunkRootLengths := make([]float64, 2)
	originalNeckRootToWristLengths := make([]float64, 2)
	originalArmToWristLengths := make([]float64, 2)
	zRatios := make([]float64, 2)
	xRatios := make([]float64, 2)
	yRatios := make([]float64, 2)

	for i, direction := range directions {
		originalTrunkRootBone, _ := sizingSet.OriginalConfigModel.Bones.GetByName(pmx.TRUNK_ROOT.String())
		originalNeckRootBone, _ := sizingSet.OriginalConfigModel.Bones.GetByName(pmx.NECK_ROOT.String())
		originalArmBone, _ := sizingSet.OriginalConfigModel.Bones.GetByName(pmx.ARM.StringFromDirection(direction))
		originalElbowBone, _ := sizingSet.OriginalConfigModel.Bones.GetByName(pmx.ELBOW.StringFromDirection(direction))
		originalWristBone, _ := sizingSet.OriginalConfigModel.Bones.GetByName(pmx.WRIST.StringFromDirection(direction))

		sizingTrunkRootBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.TRUNK_ROOT.String())
		sizingNeckRootBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.NECK_ROOT.String())
		sizingArmBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.ARM.StringFromDirection(direction))
		sizingElbowBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.ELBOW.StringFromDirection(direction))
		sizingWristBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.WRIST.StringFromDirection(direction))

		originalArmLength := originalArmBone.Position.Distance(originalTrunkRootBone.Position)
		sizingArmLength := sizingArmBone.Position.Distance(sizingTrunkRootBone.Position)

		originalElbowLength := originalElbowBone.Position.Distance(originalArmBone.Position)
		sizingElbowLength := sizingElbowBone.Position.Distance(sizingArmBone.Position)

		originalWristLength := originalWristBone.Position.Distance(originalElbowBone.Position)
		sizingWristLength := sizingWristBone.Position.Distance(sizingElbowBone.Position)

		originalNeckRootToTrunkRootLengths[i] = originalTrunkRootBone.Position.Distance(originalNeckRootBone.Position)
		originalNeckRootToWristLengths[i] = originalArmLength + originalElbowLength + originalWristLength
		originalArmToWristLengths[i] = originalElbowLength + originalWristLength

		sizingNeckRootToTrunkRootLength := sizingTrunkRootBone.Position.Distance(sizingNeckRootBone.Position)
		sizingNeckRootToWristLength := sizingArmLength + sizingElbowLength + sizingWristLength
		sizingArmToWristLength := sizingElbowLength + sizingWristLength

		yRatios[i] = sizingNeckRootToTrunkRootLength / originalNeckRootToTrunkRootLengths[i]
		xRatios[i] = sizingNeckRootToWristLength / originalNeckRootToWristLengths[i]
		zRatios[i] = sizingArmToWristLength / originalArmToWristLengths[i]
	}

	err := miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			originalTrunkRootDelta := originalAllDeltas[index].Bones.GetByName(pmx.TRUNK_ROOT.String())
			originalNeckRootDelta := originalAllDeltas[index].Bones.GetByName(pmx.NECK_ROOT.String())
			sizingTrunkRootDelta := sizingAllDeltas[index].Bones.GetByName(pmx.TRUNK_ROOT.String())
			sizingNeckRootDelta := sizingAllDeltas[index].Bones.GetByName(pmx.NECK_ROOT.String())

			for i, direction := range directions {
				armBoneName := pmx.ARM.StringFromDirection(direction)
				elbowBoneName := pmx.ELBOW.StringFromDirection(direction)
				wristBoneName := pmx.WRIST.StringFromDirection(direction)

				originalWristDelta := originalAllDeltas[index].Bones.GetByName(wristBoneName)
				sizingWristDelta := sizingAllDeltas[index].Bones.GetByName(wristBoneName)

				// ----------

				// 元の首根元から見た、手首のローカル位置
				originalWristFromNeckLocalPosition := originalNeckRootDelta.FilledGlobalMatrix().Inverted().MulVec3(originalWristDelta.FilledGlobalPosition())

				sizingWristFromNeckLocalPosition := sizingNeckRootDelta.FilledGlobalMatrix().Inverted().MulVec3(sizingWristDelta.FilledGlobalPosition())

				// 元の手首のローカル位置の比率を、距離差を考慮してスケールする
				sizingWristFromNeckLerpLocalPosition := &mmath.MVec3{
					X: mmath.Lerp(originalWristFromNeckLocalPosition.X,
						originalWristFromNeckLocalPosition.X*xRatios[i],
						math.Abs(originalWristFromNeckLocalPosition.X/originalNeckRootToWristLengths[i])),
					Y: sizingWristFromNeckLocalPosition.Y,
					Z: sizingWristFromNeckLocalPosition.Z,
				}

				// 先の首根元に手首のローカル位置を合わせたグローバル位置
				sizingWristFromNeckIdealPosition := sizingNeckRootDelta.FilledGlobalMatrix().MulVec3(sizingWristFromNeckLerpLocalPosition)

				// 元の体幹中心から見た、手首のローカル位置
				originalWristFromTrunkLocalPosition := originalTrunkRootDelta.FilledGlobalMatrix().Inverted().MulVec3(originalWristDelta.FilledGlobalPosition())

				sizingWristFromTrunkLocalPosition := sizingTrunkRootDelta.FilledGlobalMatrix().Inverted().MulVec3(sizingWristDelta.FilledGlobalPosition())

				// 元の手首のローカル位置の比率を、距離差を考慮してスケールする
				sizingWristFromTrunkLerpLocalPosition := &mmath.MVec3{
					X: mmath.Lerp(originalWristFromTrunkLocalPosition.X,
						originalWristFromTrunkLocalPosition.X*xRatios[i],
						math.Abs(originalWristFromTrunkLocalPosition.X/originalNeckRootToWristLengths[i])),
					Y: sizingWristFromTrunkLocalPosition.Y,
					Z: sizingWristFromTrunkLocalPosition.Z,
				}

				// 先の首根元に手首のローカル位置を合わせたグローバル位置
				sizingWristIFromTrunkIdealPosition := sizingTrunkRootDelta.FilledGlobalMatrix().MulVec3(sizingWristFromTrunkLerpLocalPosition)

				// 腹と首根元の中間点を求める
				// 腹（ー）に近ければ腹基準、首（＋）に近ければ首基準
				sizingWristIdealPosition := sizingWristIFromTrunkIdealPosition.Lerp(
					sizingWristFromNeckIdealPosition,
					originalWristFromTrunkLocalPosition.Y/originalNeckRootToTrunkRootLengths[i])

				// ----------

				if mlog.IsDebug() {
					originalWristPositions[i][index] = originalWristDelta.FilledGlobalPosition().Copy()

					sizingWristPositions[i][index] = sizingWristDelta.FilledGlobalPosition().Copy()
					sizingWristIdealPositions[i][index] = sizingWristIdealPosition.Copy()
					sizingWristFromNeckIdealPositions[i][index] = sizingNeckRootDelta.FilledGlobalMatrix().MulVec3(sizingWristFromNeckLocalPosition)
					sizingWristFromTrunkIdealPositions[i][index] = sizingTrunkRootDelta.FilledGlobalMatrix().MulVec3(sizingWristFromTrunkLocalPosition)
				}

				// ------------

				sizingWristDeltas := deform.DeformIk(sizingSet.SizingConfigModel, sizingProcessMotion, sizingAllDeltas[index], float32(data), armIkBones[i], sizingWristIdealPosition, all_arm_bone_names[i], false, false)

				sizingArmRotations[i][index] = sizingWristDeltas.Bones.GetByName(pmx.ARM.StringFromDirection(direction)).FilledFrameRotation()
				sizingElbowRotations[i][index] = sizingWristDeltas.Bones.GetByName(pmx.ELBOW.StringFromDirection(direction)).FilledFrameRotation()

				armBf := sizingProcessMotion.BoneFrames.Get(armBoneName).Get(float32(data))
				elbowBf := sizingProcessMotion.BoneFrames.Get(elbowBoneName).Get(float32(data))
				wristBf := sizingProcessMotion.BoneFrames.Get(wristBoneName).Get(float32(data))

				armCancelRotation := sizingArmRotations[i][index].Inverted().Muled(armBf.FilledRotation()).Normalized()
				elbowCancelRotation := sizingElbowRotations[i][index].Inverted().Muled(elbowBf.FilledRotation()).Normalized()

				sizingWristRotations[i][index] = armCancelRotation.Muled(elbowCancelRotation).Muled(wristBf.FilledRotation())
			}

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("位置合わせ02", sizingSet.Index, iterIndex, allCount)
		})
	if err != nil {
		return err
	}

	if mlog.IsDebug() {
		motion := vmd.NewVmdMotion("")

		for i, iFrame := range allFrames {
			frame := float32(iFrame)
			for j, direction := range directions {
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = originalWristPositions[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("位元%s手首", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = sizingWristPositions[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("位先%s手首", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = sizingWristFromTrunkIdealPositions[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("位先%s手首腹理", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = sizingWristFromNeckIdealPositions[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("位先%s手首首理", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = sizingWristIdealPositions[j][i]
					bf.Rotation = sizingWristRotations[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("位先%s手首理想", direction.String()), bf)
				}
			}
		}

		outputVerboseMotion("位置02", sizingSet.OutputMotionPath, motion)
	}

	incrementCompletedCount()

	// 肩回転をサイジング先モーションに反映
	updateWrist(sizingSet, allFrames, sizingProcessMotion, sizingArmRotations, sizingElbowRotations, sizingWristRotations)

	return nil
}

func createArmIkBone(sizingSet *domain.SizingSet, direction pmx.BoneDirection) *pmx.Bone {
	// 腕IK
	armBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.ARM.StringFromDirection(direction))
	elbowBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.ELBOW.StringFromDirection(direction))
	wristBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.WRIST.StringFromDirection(direction))

	ikBone := pmx.NewBoneByName(fmt.Sprintf("%s%sIk", pmx.MLIB_PREFIX, armBone.Name()))
	ikBone.Position = wristBone.Position.Copy()
	ikBone.Ik = pmx.NewIk()
	ikBone.Ik.BoneIndex = wristBone.Index()
	ikBone.Ik.LoopCount = 100
	ikBone.Ik.UnitRotation = &mmath.MVec3{X: 0.1, Y: 0.0, Z: 0.0}
	ikBone.Ik.Links = make([]*pmx.IkLink, 0)

	for _, boneIndex := range wristBone.ParentBoneIndexes {
		parentBone, _ := sizingSet.SizingConfigModel.Bones.Get(boneIndex)
		link := pmx.NewIkLink()
		link.BoneIndex = parentBone.Index()
		if parentBone.Name() != armBone.Name() && parentBone.Name() != elbowBone.Name() {
			// 腕・ひじ以外は動かさない
			link.AngleLimit = true
		}
		ikBone.Ik.Links = append(ikBone.Ik.Links, link)

		if parentBone.Name() == armBone.Name() {
			// 腕までいったら終了
			break
		}
	}

	return ikBone
}

// updateWrist は、補正した下半身回転をサイジング先モーションに反映します。
func updateWrist(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	sizingArmRotations, sizingElbowRotations, sizingWristRotations [][]*mmath.MQuaternion,
) {
	for i, iFrame := range allFrames {
		frame := float32(iFrame)
		for j, direction := range directions {
			{
				bf := sizingProcessMotion.BoneFrames.Get(pmx.ARM.StringFromDirection(direction)).Get(frame)
				bf.Rotation = sizingArmRotations[j][i]
				sizingProcessMotion.InsertBoneFrame(pmx.ARM.StringFromDirection(direction), bf)
			}
			{
				bf := sizingProcessMotion.BoneFrames.Get(pmx.ELBOW.StringFromDirection(direction)).Get(frame)
				bf.Rotation = sizingElbowRotations[j][i]
				sizingProcessMotion.InsertBoneFrame(pmx.ELBOW.StringFromDirection(direction), bf)
			}
			{
				bf := sizingProcessMotion.BoneFrames.Get(pmx.WRIST.StringFromDirection(direction)).Get(frame)
				bf.Rotation = sizingWristRotations[j][i]
				sizingProcessMotion.InsertBoneFrame(pmx.WRIST.StringFromDirection(direction), bf)
			}
		}

		if i > 0 && i%1000 == 0 {
			processLog("位置補正03", sizingSet.Index, i, len(allFrames))
		}
	}

	if mlog.IsDebug() {
		outputVerboseMotion("位置03", sizingSet.OutputMotionPath, sizingProcessMotion)
	}
}

func updateWristResultMotion(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, sizingProcessMotion *vmd.VmdMotion,
	incrementCompletedCount func(),
) error {
	// 肩補正処理の結果をサイジング先モーションに反映
	sizingModel := sizingSet.SizingConfigModel
	outputMotion := sizingSet.OutputMotion

	activeFrames := getFrames(outputMotion, []string{pmx.ARM.Left(), pmx.ARM.Right(),
		pmx.ELBOW.Left(), pmx.ELBOW.Right(), pmx.WRIST.Left(), pmx.WRIST.Right()})

	// 腕系はあるボーンだけ上書きする
	for _, boneName := range []string{
		pmx.ARM.Left(), pmx.ARM.Right(), pmx.ELBOW.Left(), pmx.ELBOW.Right(), pmx.WRIST.Left(), pmx.WRIST.Right(),
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
	threshold := 0.1

	err := miter.IterParallelByList(directions, 1, 1,
		func(dIndex int, direction pmx.BoneDirection) error {
			for tIndex, targetFrames := range [][]int{activeFrames, allFrames} {
				processAllDeltas, err := computeVmdDeltas(targetFrames, blockSize,
					sizingModel, sizingProcessMotion, sizingSet, true, all_arm_bone_names[dIndex], "位補正01")
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
						sizingModel, outputMotion, sizingSet, true, all_arm_bone_names[dIndex], "")
					if err != nil {
						return err
					}

					// 腕の位置をチェック
					resultArmDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.ARM.StringFromDirection(direction))
					processArmDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.ARM.StringFromDirection(direction))

					resultElbowDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.ELBOW.StringFromDirection(direction))
					processElbowDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.ELBOW.StringFromDirection(direction))

					resultWristDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.WRIST.StringFromDirection(direction))
					processWristDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.WRIST.StringFromDirection(direction))

					// 各関節位置がズレている場合、元の回転を焼き込む
					if resultArmDelta.FilledGlobalPosition().Distance(processArmDelta.FilledGlobalPosition()) > threshold ||
						resultElbowDelta.FilledGlobalPosition().Distance(processElbowDelta.FilledGlobalPosition()) > threshold ||
						resultWristDelta.FilledGlobalPosition().Distance(processWristDelta.FilledGlobalPosition()) > threshold {
						for _, boneName := range []string{
							pmx.ARM.StringFromDirection(direction),
							pmx.ELBOW.StringFromDirection(direction),
							pmx.WRIST.StringFromDirection(direction),
						} {
							processBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
							resultBf := outputMotion.BoneFrames.Get(boneName).Get(frame)
							resultBf.Rotation = processBf.FilledRotation().Copy()
							outputMotion.InsertBoneFrame(boneName, resultBf)
						}
					}

					if fIndex > 0 && fIndex%1000 == 0 {
						mlog.I(mi18n.T("位置補正04", map[string]interface{}{
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
	if err != nil {
		return err
	}

	if mlog.IsDebug() {
		outputVerboseMotion("位置04", sizingSet.OutputMotionPath, outputMotion)
	}

	return err
}

func checkBonesForSizingWrist(sizingSet *domain.SizingSet) (err error) {

	for _, v := range [][]interface{}{
		{sizingSet.OriginalNeckRootBone, pmx.NECK_ROOT.String(), false},
		{sizingSet.OriginalLeftArmBone, pmx.ARM.Left(), true},
		{sizingSet.OriginalLeftElbowBone, pmx.ELBOW.Left(), true},
		{sizingSet.OriginalLeftWristBone, pmx.WRIST.Left(), true},
		{sizingSet.OriginalRightArmBone, pmx.ARM.Right(), true},
		{sizingSet.OriginalRightElbowBone, pmx.ELBOW.Right(), true},
		{sizingSet.OriginalRightWristBone, pmx.WRIST.Right(), true},
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
				"Process": mi18n.T("手首位置合わせ"), "No": sizingSet.Index + 1, "ModelType": "元モデル", "BoneName": boneName}))
			err = merr.NameNotFoundError
		}
	}

	// ------------------------------------------

	for _, v := range [][]interface{}{
		{sizingSet.SizingNeckRootBone, pmx.NECK_ROOT.String(), false},
		{sizingSet.SizingLeftArmBone, pmx.ARM.Left(), true},
		{sizingSet.SizingLeftElbowBone, pmx.ELBOW.Left(), true},
		{sizingSet.SizingLeftWristBone, pmx.WRIST.Left(), true},
		{sizingSet.SizingRightArmBone, pmx.ARM.Right(), true},
		{sizingSet.SizingRightElbowBone, pmx.ELBOW.Right(), true},
		{sizingSet.SizingRightWristBone, pmx.WRIST.Right(), true},
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
