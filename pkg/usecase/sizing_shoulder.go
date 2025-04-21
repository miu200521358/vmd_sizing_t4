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

type SizingShoulderUsecase struct {
}

func NewSizingShoulderUsecase() *SizingShoulderUsecase {
	return &SizingShoulderUsecase{}
}

// SizingShoulder は肩補正処理を行います。
func (su *SizingShoulderUsecase) Exec(sizingSet *domain.SizingSet, sizingSetCount int, incrementCompletedCount func()) (bool, error) {
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
	if err := su.checkBones(sizingSet); err != nil {
		return false, err
	}

	allFrames := mmath.IntRanges(int(originalMotion.MaxFrame()) + 1)
	blockSize, _ := miter.GetBlockSize(len(allFrames) * sizingSetCount)

	sizingAllDeltas, err := computeVmdDeltas(allFrames, blockSize, sizingSet.SizingConfigModel, sizingProcessMotion, sizingSet, true, append(all_arm_bone_names[0], all_arm_bone_names[1]...), "肩補正01")
	if err != nil {
		return false, err
	}

	incrementCompletedCount()

	if err := su.calculateAdjustedShoulder(sizingSet, allFrames, blockSize, sizingAllDeltas, sizingProcessMotion, incrementCompletedCount); err != nil {
		return false, err
	}

	if err := su.updateOutputMotion(sizingSet, allFrames, blockSize, sizingProcessMotion,
		incrementCompletedCount); err != nil {
		return false, err
	}

	sizingSet.CompletedSizingShoulder = true
	sizingSet.CompletedShoulderWeight = sizingSet.ShoulderWeight

	return true, nil
}

func (su *SizingShoulderUsecase) createShoulderIkBone(sizingSet *domain.SizingSet, direction pmx.BoneDirection) *pmx.Bone {
	// 肩IK
	shoulderBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.SHOULDER.StringFromDirection(direction))
	armBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.ARM.StringFromDirection(direction))

	ikBone := pmx.NewBoneByName(fmt.Sprintf("%s%sIk", pmx.MLIB_PREFIX, shoulderBone.Name()))
	ikBone.Position = armBone.Position.Copy()
	ikBone.Ik = pmx.NewIk()
	ikBone.Ik.BoneIndex = armBone.Index()
	ikBone.Ik.LoopCount = 100
	ikBone.Ik.UnitRotation = &mmath.MVec3{X: 0.1, Y: 0.0, Z: 0.0}
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
			// 肩までいったら終了
			break
		}
	}

	return ikBone
}

func (su *SizingShoulderUsecase) createElbowIk(sizingSet *domain.SizingSet, direction pmx.BoneDirection) *pmx.Bone {
	// ひじIK
	armBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.ARM.StringFromDirection(direction))
	elbowBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.ELBOW.StringFromDirection(direction))

	ikBone := pmx.NewBoneByName(fmt.Sprintf("%s%sIk", pmx.MLIB_PREFIX, elbowBone.Name()))
	ikBone.Position = elbowBone.Position.Copy()
	ikBone.Ik = pmx.NewIk()
	ikBone.Ik.BoneIndex = elbowBone.Index()
	ikBone.Ik.LoopCount = 100
	ikBone.Ik.UnitRotation = &mmath.MVec3{X: 0.1, Y: 0.0, Z: 0.0}
	ikBone.Ik.Links = make([]*pmx.IkLink, 0)
	for _, boneIndex := range elbowBone.ParentBoneIndexes {
		parentBone, _ := sizingSet.SizingConfigModel.Bones.Get(boneIndex)
		link := pmx.NewIkLink()
		link.BoneIndex = parentBone.Index()
		if parentBone.Name() != armBone.Name() {
			// 腕以外は動かさない
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

func (su *SizingShoulderUsecase) createWristIkBone(sizingSet *domain.SizingSet, direction pmx.BoneDirection) *pmx.Bone {
	// 手首IK
	armBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.ARM.StringFromDirection(direction))
	wristBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.WRIST.StringFromDirection(direction))

	ikBone := pmx.NewBoneByName(fmt.Sprintf("%s%sIk", pmx.MLIB_PREFIX, wristBone.Name()))
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
		if parentBone.Name() != armBone.Name() {
			// 腕以外は動かさない
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

func (su *SizingShoulderUsecase) calculateAdjustedShoulder(
	sizingSet *domain.SizingSet, allFrames []int, blockSize int,
	sizingAllDeltas []*delta.VmdDeltas, sizingProcessMotion *vmd.VmdMotion,
	incrementCompletedCount func(),
) error {
	outputVerboseMotion("肩01", sizingSet.OutputMotionPath, sizingProcessMotion)

	shoulderIkBones := make([]*pmx.Bone, 2)
	elbowIkBones := make([]*pmx.Bone, 2)
	wristIkBones := make([]*pmx.Bone, 2)
	for i, direction := range directions {
		shoulderIkBones[i] = su.createShoulderIkBone(sizingSet, direction)
		elbowIkBones[i] = su.createElbowIk(sizingSet, direction)
		wristIkBones[i] = su.createWristIkBone(sizingSet, direction)
	}

	armPositions := make([][]*mmath.MVec3, 2)
	armIdealPositions := make([][]*mmath.MVec3, 2)
	armRatioPositions := make([][]*mmath.MVec3, 2)
	armFixYPositions := make([][]*mmath.MVec3, 2)
	armLimitedPositions := make([][]*mmath.MVec3, 2)
	shoulderResultPositions := make([][]*mmath.MVec3, 2)
	shoulderResultRotations := make([][]*mmath.MQuaternion, 2)
	armResultPositions := make([][]*mmath.MVec3, 2)
	armResultRotations := make([][]*mmath.MQuaternion, 2)
	elbowResultPositions := make([][]*mmath.MVec3, 2)
	elbowResultRotations := make([][]*mmath.MQuaternion, 2)
	wristPositions := make([][]*mmath.MVec3, 2)
	wristResultPositions := make([][]*mmath.MVec3, 2)
	wristResultRotations := make([][]*mmath.MQuaternion, 2)

	armLocalInitialPositions := make([]*mmath.MVec3, 2)
	armRatios := make([]*mmath.MVec3, 2) // ひじまでの長さに対する腕までの長さの割合

	shoulderWeight := float64(sizingSet.ShoulderWeight) / 100.0

	for i := range directions {
		armPositions[i] = make([]*mmath.MVec3, len(allFrames))
		armIdealPositions[i] = make([]*mmath.MVec3, len(allFrames))
		armRatioPositions[i] = make([]*mmath.MVec3, len(allFrames))
		armFixYPositions[i] = make([]*mmath.MVec3, len(allFrames))
		armLimitedPositions[i] = make([]*mmath.MVec3, len(allFrames))
		shoulderResultPositions[i] = make([]*mmath.MVec3, len(allFrames))
		shoulderResultRotations[i] = make([]*mmath.MQuaternion, len(allFrames))
		armResultPositions[i] = make([]*mmath.MVec3, len(allFrames))
		armResultRotations[i] = make([]*mmath.MQuaternion, len(allFrames))
		elbowResultPositions[i] = make([]*mmath.MVec3, len(allFrames))
		elbowResultRotations[i] = make([]*mmath.MQuaternion, len(allFrames))
		wristPositions[i] = make([]*mmath.MVec3, len(allFrames))
		wristResultPositions[i] = make([]*mmath.MVec3, len(allFrames))
		wristResultRotations[i] = make([]*mmath.MQuaternion, len(allFrames))

		neckRootBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.NECK_ROOT.String())
		armBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.ARM.StringFromDirection(directions[i]))
		elbowBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.ELBOW.StringFromDirection(directions[i]))

		armLocalInitialPositions[i] = armBone.Position.Subed(neckRootBone.Position)

		armLength := armLocalInitialPositions[i].Length()
		elbowLength := armLocalInitialPositions[i].Length() + elbowBone.Position.Distance(armBone.Position)
		armRatios[i] = &mmath.MVec3{
			X: armLength / elbowLength,
			Y: armLength / elbowLength,
			Z: armLength / elbowLength,
		}
	}

	err := miter.IterParallelByList(allFrames, blockSize, log_block_size,
		func(index, data int) error {
			if sizingSet.IsTerminate {
				return merr.TerminateError
			}

			frame := float32(data)
			sizingNeckRootDelta := sizingAllDeltas[index].Bones.GetByName(pmx.NECK_ROOT.String())

			for i, direction := range directions {
				armBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.ARM.StringFromDirection(direction))
				elbowBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.ELBOW.StringFromDirection(direction))
				wristBone, _ := sizingSet.SizingConfigModel.Bones.GetByName(pmx.WRIST.StringFromDirection(direction))

				sizingArmDelta := sizingAllDeltas[index].Bones.GetByName(armBone.Name())
				sizingElbowDelta := sizingAllDeltas[index].Bones.GetByName(elbowBone.Name())
				sizingWristDelta := sizingAllDeltas[index].Bones.GetByName(wristBone.Name())

				// 先の首根元から見た、先腕のローカル位置
				armLocalPositionFromNeckRoot := sizingNeckRootDelta.FilledGlobalMatrix().Inverted().MulVec3(sizingArmDelta.FilledGlobalPosition())
				// 先の首根元からみた、先ひじのローカル位置
				elbowLocalPositionFromNeckRoot := sizingNeckRootDelta.FilledGlobalMatrix().Inverted().MulVec3(sizingElbowDelta.FilledGlobalPosition())

				armLocalInitialPosition := armLocalInitialPositions[i]

				// ひじのローカル位置から比率で腕のローカル位置を求める
				armLocalPositionFromRatio := elbowLocalPositionFromNeckRoot.Muled(armRatios[i])

				// 腕のローカルYはひじだけ上げる動作があり得るので、元々の腕Yと比率で求めた腕Yの中間を取る
				// 肩だけ上げている場合、t がマイナスになるので、元々の腕Yになる
				armX := mmath.Lerp(armLocalPositionFromNeckRoot.X, armLocalPositionFromRatio.X,
					armLocalPositionFromNeckRoot.X/armRatios[i].X/elbowLocalPositionFromNeckRoot.X)
				armY := mmath.Lerp(armLocalPositionFromNeckRoot.Y, armLocalPositionFromRatio.Y,
					armLocalPositionFromNeckRoot.Y/armRatios[i].Y/elbowLocalPositionFromNeckRoot.Y)
				armZ := mmath.Lerp(armLocalPositionFromNeckRoot.Z, armLocalPositionFromRatio.Z,
					armLocalPositionFromNeckRoot.Z/armRatios[i].Z/elbowLocalPositionFromNeckRoot.Z)

				// 腕Yが決まった、腕のローカル位置
				armLocalPositionFixY := &mmath.MVec3{X: armX, Y: armY, Z: armZ}

				// 肩のウェイトに合わせて移動量を決める
				sizingArmLocalPosition := armLocalInitialPosition.Slerp(armLocalPositionFixY, shoulderWeight)

				// 元の首根元に先の腕のローカル位置を合わせたグローバル位置
				sizingArmIdealPosition := sizingNeckRootDelta.FilledGlobalMatrix().MulVec3(sizingArmLocalPosition)

				sizingElbowPosition := sizingElbowDelta.FilledGlobalPosition().Copy()
				sizingWristPosition := sizingWristDelta.FilledGlobalPosition().Copy()

				if mlog.IsDebug() {
					armPositions[i][index] = sizingArmDelta.FilledGlobalPosition().Copy()
					armIdealPositions[i][index] = sizingArmIdealPosition.Copy()
					armRatioPositions[i][index] = sizingNeckRootDelta.FilledGlobalMatrix().MulVec3(armLocalPositionFromRatio)
					armFixYPositions[i][index] = sizingNeckRootDelta.FilledGlobalMatrix().MulVec3(armLocalPositionFixY)

					wristPositions[i][index] = sizingWristPosition.Copy()
				}

				sizingShoulderDeltas := deform.DeformIks(sizingSet.SizingConfigModel, sizingProcessMotion,
					sizingAllDeltas[index], frame, []*pmx.Bone{shoulderIkBones[i], elbowIkBones[i], wristIkBones[i]},
					[]*pmx.Bone{armBone, elbowBone, wristBone},
					[]*mmath.MVec3{sizingArmIdealPosition, sizingElbowPosition, sizingWristPosition},
					all_arm_bone_names[i], 5, false, false)

				shoulderResultRotations[i][index] = sizingShoulderDeltas.Bones.GetByName(pmx.SHOULDER.StringFromDirection(direction)).FilledFrameRotation()
				armResultRotations[i][index] = sizingShoulderDeltas.Bones.GetByName(armBone.Name()).FilledFrameRotation()

				if mlog.IsDebug() {
					shoulderDeformDelta := sizingShoulderDeltas.Bones.GetByName(pmx.SHOULDER.StringFromDirection(direction))
					shoulderResultPositions[i][index] = shoulderDeformDelta.FilledGlobalPosition().Copy()

					armDeformDelta := sizingShoulderDeltas.Bones.GetByName(pmx.ARM.StringFromDirection(direction))
					armResultPositions[i][index] = armDeformDelta.FilledGlobalPosition().Copy()

					elbowDeformDelta := sizingShoulderDeltas.Bones.GetByName(pmx.ELBOW.StringFromDirection(direction))
					elbowResultPositions[i][index] = elbowDeformDelta.FilledGlobalPosition().Copy()
					elbowResultRotations[i][index] = elbowDeformDelta.FilledFrameRotation().Copy()

					wristDeformDelta := sizingShoulderDeltas.Bones.GetByName(pmx.WRIST.StringFromDirection(direction))
					wristResultPositions[i][index] = wristDeformDelta.FilledGlobalPosition().Copy()
					wristResultRotations[i][index] = wristDeformDelta.FilledFrameRotation().Copy()
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
					motion.InsertBoneFrame(fmt.Sprintf("%s腕先肩", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = armRatioPositions[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("%s腕比率先肩", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = armFixYPositions[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("%s腕Y固定先肩", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = armLimitedPositions[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("%s腕限定先肩", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = armIdealPositions[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("%s腕理想先肩", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = wristPositions[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("%s手首先肩", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = shoulderResultPositions[j][i]
					bf.Rotation = shoulderResultRotations[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("%s肩結果先肩", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = armResultPositions[j][i]
					bf.Rotation = armResultRotations[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("%s腕結果先肩", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = elbowResultPositions[j][i]
					bf.Rotation = elbowResultRotations[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("%sひじ結果先肩", direction.String()), bf)
				}
				{
					bf := vmd.NewBoneFrame(frame)
					bf.Position = wristResultPositions[j][i]
					bf.Rotation = wristResultRotations[j][i]
					motion.InsertBoneFrame(fmt.Sprintf("%s手首結果先肩", direction.String()), bf)
				}
			}
		}

		outputVerboseMotion("肩02", sizingSet.OutputMotionPath, motion)
	}

	incrementCompletedCount()

	// 肩回転をサイジング先モーションに反映
	su.updateShoulder(sizingSet, allFrames, sizingProcessMotion, shoulderResultRotations, armResultRotations)

	incrementCompletedCount()

	return nil
}

// updateShoulder は、補正した下半身回転をサイジング先モーションに反映します。
func (su *SizingShoulderUsecase) updateShoulder(
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

func (su *SizingShoulderUsecase) updateOutputMotion(
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
					sizingModel, sizingProcessMotion, sizingSet, true, all_arm_bone_names[dIndex], "肩補正01")
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
						sizingModel, outputMotion, sizingSet, true, all_arm_bone_names[dIndex], "")
					if err != nil {
						return err
					}

					// 腕の位置をチェック
					resultArmDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.ARM.StringFromDirection(direction))
					processArmDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.ARM.StringFromDirection(direction))

					// ひじの位置をチェック
					resultElbowDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.ELBOW.StringFromDirection(direction))
					processElbowDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.ELBOW.StringFromDirection(direction))

					// 手首の位置をチェック
					resultWristDelta := resultAllVmdDeltas[0].Bones.GetByName(pmx.WRIST.StringFromDirection(direction))
					processWristDelta := processAllDeltas[fIndex].Bones.GetByName(pmx.WRIST.StringFromDirection(direction))

					// 各関節位置がズレている場合、元の回転を焼き込む
					if resultArmDelta.FilledGlobalPosition().Distance(processArmDelta.FilledGlobalPosition()) > threshold ||
						resultElbowDelta.FilledGlobalPosition().Distance(processElbowDelta.FilledGlobalPosition()) > threshold ||
						resultWristDelta.FilledGlobalPosition().Distance(processWristDelta.FilledGlobalPosition()) > threshold {
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

					if fIndex > 0 && int(iFrame/1000) > prevLog {
						mlog.I(mi18n.T("肩補正04", map[string]interface{}{
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
		outputVerboseMotion("肩04", sizingSet.OutputMotionPath, outputMotion)
	}

	return err
}

func (su *SizingShoulderUsecase) checkBones(sizingSet *domain.SizingSet) (err error) {

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
