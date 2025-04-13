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

// SizingShoulder は肩補正処理を行います。
func SizingShoulder(sizingSet *domain.SizingSet, sizingSetCount int, incrementCompletedCount func()) (bool, error) {
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

	// // 元モデルのデフォーム結果を並列処理で取得
	// originalAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.OriginalConfigModel, originalMotion, sizingSet, true, append(all_shoulder_bone_names[0], all_shoulder_bone_names[1]...), "肩補正01")
	// if err != nil {
	// 	return false, err
	// }

	// incrementCompletedCount()

	sizingAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel, sizingProcessMotion, sizingSet, true, append(all_shoulder_bone_names[0], all_shoulder_bone_names[1]...), "肩補正01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	if err := calculateAdjustedShoulder(sizingSet, allFrames, blockSize, sizingAllDeltas, sizingProcessMotion, incrementCompletedCount); err != nil {
		return false, err
	}

	incrementCompletedCount()

	if err := updateShoulderResultMotion(sizingSet, allFrames, blockSize, sizingProcessMotion,
		incrementCompletedCount); err != nil {
		return false, err
	}

	incrementCompletedCount()

	sizingSet.CompletedSizingShoulder = true
	sizingSet.CompletedShoulderWeight = sizingSet.ShoulderWeight

	return true, nil
}

func createShoulderIkBone(sizingSet *domain.SizingSet, direction pmx.BoneDirection) *pmx.Bone {
	// 肩IK
	shoulderBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.SHOULDER.StringFromDirection(direction))
	armBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.ARM.StringFromDirection(direction))

	ikBone := pmx.NewBoneByName(fmt.Sprintf("%s%sIk", pmx.MLIB_PREFIX, shoulderBone.Name()))
	ikBone.Position = armBone.Position.Copy()
	ikBone.Ik = pmx.NewIk()
	ikBone.Ik.BoneIndex = armBone.Index()
	ikBone.Ik.LoopCount = 10
	ikBone.Ik.UnitRotation = &mmath.MVec3{X: 2, Y: 0.0, Z: 0.0}
	ikBone.Ik.Links = make([]*pmx.IkLink, 0)
	for _, boneIndex := range armBone.ParentBoneIndexes {
		parentBone, _ := sizingSet.SizingConfigModel.Bones.Get(boneIndex)
		link := pmx.NewIkLink()
		link.BoneIndex = parentBone.Index()
		if parentBone.Name() != shoulderBone.Name() {
			// 肩以外は動かさない
			link.AngleLimit = true
		}
		ikBone.Ik.Links = append(ikBone.Ik.Links, link)

		if parentBone.Name() == shoulderBone.Name() {
			// 腕までいったら終了
			break
		}
	}

	return ikBone
}

func calculateAdjustedShoulder(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int,
	sizingAllDeltas []*delta.VmdDeltas, sizingProcessMotion *vmd.VmdMotion,
	incrementCompletedCount func(),
) error {
	outputVerboseMotion("肩01", sizingSet.OutputMotionPath, sizingProcessMotion)

	shoulderIkBones := make([]*pmx.Bone, 2)
	for i, direction := range directions {
		shoulderIkBones[i] = createShoulderIkBone(sizingSet, direction)
	}

	armPositions := make([][]*mmath.MVec3, 2)
	armIdealPositions := make([][]*mmath.MVec3, 2)
	armRatioPositions := make([][]*mmath.MVec3, 2)
	armFixYPositions := make([][]*mmath.MVec3, 2)
	armLimitedPositions := make([][]*mmath.MVec3, 2)
	elbowPositions := make([][]*mmath.MVec3, 2)
	shoulderPositions := make([][]*mmath.MVec3, 2)
	shoulderRotations := make([][]*mmath.MQuaternion, 2)
	armRotations := make([][]*mmath.MQuaternion, 2)

	armLocalInitialPositions := make([]*mmath.MVec3, 2)
	armRatios := make([]*mmath.MVec3, 2) // ひじまでの長さに対する腕までの長さの割合

	shoulderWeight := float64(sizingSet.ShoulderWeight) / 100.0

	for i := range directions {
		armPositions[i] = make([]*mmath.MVec3, len(allFrames))
		armIdealPositions[i] = make([]*mmath.MVec3, len(allFrames))
		armRatioPositions[i] = make([]*mmath.MVec3, len(allFrames))
		armFixYPositions[i] = make([]*mmath.MVec3, len(allFrames))
		armLimitedPositions[i] = make([]*mmath.MVec3, len(allFrames))
		elbowPositions[i] = make([]*mmath.MVec3, len(allFrames))
		shoulderPositions[i] = make([]*mmath.MVec3, len(allFrames))
		shoulderRotations[i] = make([]*mmath.MQuaternion, len(allFrames))
		armRotations[i] = make([]*mmath.MQuaternion, len(allFrames))

		neckRootBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.NECK_ROOT.String())
		armBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.ARM.StringFromDirection(directions[i]))
		elbowBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.ELBOW.StringFromDirection(directions[i]))
		wristBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.WRIST.StringFromDirection(directions[i]))

		armLocalInitialPositions[i] = armBone.Position.Subed(neckRootBone.Position)

		armLength := armLocalInitialPositions[i].Length()
		elbowLength := armLocalInitialPositions[i].Length() + elbowBone.Position.Distance(armBone.Position)
		wristLength := elbowLength + wristBone.Position.Distance(elbowBone.Position)
		armRatios[i] = &mmath.MVec3{
			X: armLength / elbowLength,
			Y: armLength / wristLength,
			Z: armLength / elbowLength,
		}
	}

	err := miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			sizingNeckRootDelta := sizingAllDeltas[index].Bones.GetByName(pmx.NECK_ROOT.String())

			for i, direction := range directions {
				sizingElbowDelta := sizingAllDeltas[index].Bones.GetByName(pmx.ELBOW.StringFromDirection(direction))
				sizingArmDelta := sizingAllDeltas[index].Bones.GetByName(pmx.ARM.StringFromDirection(direction))

				// 先の首根元から見た、先腕のローカル位置
				armLocalPositionFromNeckRoot := sizingNeckRootDelta.FilledGlobalMatrix().Inverted().MulVec3(sizingArmDelta.FilledGlobalPosition())
				// 先の首根元からみた、先ひじのローカル位置
				elbowLocalPositionFromNeckRoot := sizingNeckRootDelta.FilledGlobalMatrix().Inverted().MulVec3(sizingElbowDelta.FilledGlobalPosition())

				armLocalInitialPosition := armLocalInitialPositions[i]

				// ひじのローカル位置から比率で腕のローカル位置を求める
				armLocalPositionFromRatio := elbowLocalPositionFromNeckRoot.Muled(armRatios[i])

				// 腕のローカルYはひじだけ上げる動作があり得るので、元々の腕Yと比率で求めた腕Yの中間を取る
				// ひじだけ上げている場合、t がマイナスになるので、元々の腕Yになる
				armY := mmath.Lerp(armLocalPositionFromNeckRoot.Y, armLocalPositionFromRatio.Y,
					armLocalPositionFromNeckRoot.Y/elbowLocalPositionFromNeckRoot.Y)

				// 腕Yが決まった、腕のローカル位置
				armLocalPositionFixY := &mmath.MVec3{
					X: armLocalPositionFromRatio.X,
					Y: armY,
					Z: armLocalPositionFromRatio.Z,
				}

				// 肩のウェイトに合わせて移動量を決める
				sizingArmLocalPosition := armLocalInitialPosition.Lerp(armLocalPositionFixY, shoulderWeight)

				// 元の首根元に先の腕のローカル位置を合わせたグローバル位置
				sizingArmIdealPosition := sizingNeckRootDelta.FilledGlobalMatrix().MulVec3(sizingArmLocalPosition)

				if mlog.IsDebug() {
					armPositions[i][index] = sizingArmDelta.FilledGlobalPosition().Copy()
					armIdealPositions[i][index] = sizingArmIdealPosition.Copy()
					armRatioPositions[i][index] = sizingNeckRootDelta.FilledGlobalMatrix().MulVec3(armLocalPositionFromRatio)
					armFixYPositions[i][index] = sizingNeckRootDelta.FilledGlobalMatrix().MulVec3(armLocalPositionFixY)
				}

				sizingArmDeltas := deform.DeformIk(sizingSet.SizingConfigModel, sizingProcessMotion, sizingAllDeltas[index], float32(data), shoulderIkBones[i], sizingArmIdealPosition, all_shoulder_bone_names[i], false)

				shoulderBf := sizingProcessMotion.BoneFrames.Get(pmx.SHOULDER.StringFromDirection(direction)).Get(float32(data))

				shoulderRotations[i][index] = sizingArmDeltas.Bones.GetByName(pmx.SHOULDER.StringFromDirection(direction)).FilledFrameRotation()
				shoulderCancelRotation := shoulderRotations[i][index].Inverted().Muled(shoulderBf.FilledRotation()).Normalized()

				armBf := sizingProcessMotion.BoneFrames.Get(pmx.ARM.StringFromDirection(direction)).Get(float32(data))
				armRotations[i][index] = shoulderCancelRotation.Muled(armBf.FilledRotation()).Normalized()

				if mlog.IsDebug() {
					sizingDeformShoulderDelta := sizingArmDeltas.Bones.GetByName(pmx.SHOULDER.StringFromDirection(direction))
					shoulderPositions[i][index] = sizingDeformShoulderDelta.FilledGlobalPosition()
				}
			}

			return nil
		},
		func(iterIndex, allCount int) {
			processLog("肩補正02", sizingSet.Index, iterIndex, allCount)
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
					bf.Position = armPositions[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("先%s腕", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = armRatioPositions[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("先%s腕比率", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = armFixYPositions[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("先%s腕Y固定", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = armLimitedPositions[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("先%s腕限定", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = armIdealPositions[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("先%s腕理想", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = shoulderPositions[j][i]
					bf.Rotation = shoulderRotations[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("先%s肩結果", direction.String()), bf)
				}
			}
		}

		outputVerboseMotion("肩02", sizingSet.OutputMotionPath, motion)
	}

	incrementCompletedCount()

	// 肩回転をサイジング先モーションに反映
	updateShoulder(sizingSet, allFrames, sizingProcessMotion, shoulderRotations, armRotations)

	return nil
}

// updateShoulder は、補正した下半身回転をサイジング先モーションに反映します。
func updateShoulder(
	sizingSet *domain.SizingSet, allFrames []int, sizingProcessMotion *vmd.VmdMotion,
	shoulderRotations, armRotations [][]*mmath.MQuaternion,
) {
	for i, iFrame := range allFrames {
		frame := float32(iFrame)
		for j, direction := range directions {
			{
				bf := sizingProcessMotion.BoneFrames.Get(pmx.SHOULDER.StringFromDirection(direction)).Get(frame)
				bf.Rotation = shoulderRotations[j][i]
				sizingProcessMotion.InsertBoneFrame(pmx.SHOULDER.StringFromDirection(direction), bf)
			}
			{
				bf := sizingProcessMotion.BoneFrames.Get(pmx.ARM.StringFromDirection(direction)).Get(frame)
				bf.Rotation = armRotations[j][i]
				sizingProcessMotion.InsertBoneFrame(pmx.ARM.StringFromDirection(direction), bf)
			}
		}

		if i > 0 && i%1000 == 0 {
			processLog("肩補正03", sizingSet.Index, i, len(allFrames))
		}
	}

	if mlog.IsDebug() {
		outputVerboseMotion("肩03", sizingSet.OutputMotionPath, sizingProcessMotion)
	}
}

func updateShoulderResultMotion(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int, sizingProcessMotion *vmd.VmdMotion,
	incrementCompletedCount func(),
) error {
	// 肩補正処理の結果をサイジング先モーションに反映
	sizingModel := sizingSet.SizingConfigModel
	outputMotion := sizingSet.OutputMotion

	activeFrames := getFrames(outputMotion, []string{pmx.SHOULDER.Left(), pmx.SHOULDER.Right()})

	// 肩はあるボーンだけ上書きする
	for _, boneName := range []string{
		pmx.SHOULDER.Left(), pmx.SHOULDER.Right(), pmx.ARM.Left(), pmx.ARM.Right(),
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
					sizingModel, sizingProcessMotion, sizingSet, true, all_shoulder_bone_names[dIndex], "肩補正01")
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
						sizingModel, outputMotion, sizingSet, true, all_shoulder_bone_names[dIndex], "")
					if err != nil {
						return err
					}

					// 腕の位置をチェック
					resultArmDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.ARM.StringFromDirection(direction))
					processArmDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.ARM.StringFromDirection(direction))

					// ひじの位置をチェック
					resultElbowDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.ELBOW.StringFromDirection(direction))
					processElbowDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.ELBOW.StringFromDirection(direction))

					// 各関節位置がズレている場合、元の回転を焼き込む
					if resultArmDelta.FilledGlobalPosition().Distance(processArmDelta.FilledGlobalPosition()) > threshold ||
						resultElbowDelta.FilledGlobalPosition().Distance(processElbowDelta.FilledGlobalPosition()) > threshold {
						for _, boneName := range []string{
							pmx.SHOULDER.StringFromDirection(direction),
							pmx.ARM.StringFromDirection(direction),
						} {
							processBf := sizingProcessMotion.BoneFrames.Get(boneName).Get(frame)
							resultBf := outputMotion.BoneFrames.Get(boneName).Get(frame)
							resultBf.Rotation = processBf.FilledRotation().Copy()
							outputMotion.InsertBoneFrame(boneName, resultBf)
						}
					}

					if fIndex > 0 && fIndex%1000 == 0 {
						mlog.I(mi18n.T("肩補正04", map[string]interface{}{
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
		outputVerboseMotion("肩04", sizingSet.OutputMotionPath, outputMotion)
	}

	return err
}

func checkBonesForSizingShoulder(sizingSet *domain.SizingSet) (err error) {

	for _, v := range [][]interface{}{
		{sizingSet.OriginalNeckRootBone, pmx.NECK_ROOT.String(), false},
		{sizingSet.OriginalLeftShoulderBone, pmx.SHOULDER.Left(), true},
		{sizingSet.OriginalLeftArmBone, pmx.ARM.Left(), true},
		{sizingSet.OriginalRightShoulderBone, pmx.SHOULDER.Right(), true},
		{sizingSet.OriginalRightArmBone, pmx.ARM.Right(), true},
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
		{sizingSet.SizingRightShoulderBone, pmx.SHOULDER.Right(), true},
		{sizingSet.SizingRightArmBone, pmx.ARM.Right(), true},
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
