package domain

import (
	"fmt"
	"strings"
	"sync"

	"github.com/miu200521358/mlib_go/pkg/config/merr"
	"github.com/miu200521358/mlib_go/pkg/config/mi18n"
	"github.com/miu200521358/mlib_go/pkg/config/mlog"
	"github.com/miu200521358/mlib_go/pkg/domain/mmath"
	"github.com/miu200521358/mlib_go/pkg/domain/pmx"
	"github.com/miu200521358/mlib_go/pkg/domain/vmd"
	"github.com/miu200521358/mlib_go/pkg/infrastructure/mfile"
	"github.com/miu200521358/mlib_go/pkg/infrastructure/repository"
)

type SizingSet struct {
	Index       int  // インデックス
	IsTerminate bool // 処理停止フラグ

	OriginalMotionPath string `json:"original_motion_path"` // 元モーションパス
	OriginalModelPath  string `json:"original_model_path"`  // 元モデルパス
	SizingModelPath    string `json:"sizing_model_path"`    // サイジング先モデルパス
	OutputMotionPath   string `json:"-"`                    // 出力モーションパス
	OutputModelPath    string `json:"-"`                    // 出力モデルパス

	OriginalMotionName string `json:"-"` // 元モーション名
	OriginalModelName  string `json:"-"` // 元モーション名
	OutputModelName    string `json:"-"` // サイジング先モデル名

	OriginalMotion      *vmd.VmdMotion `json:"-"` // 元モデル
	OriginalModel       *pmx.PmxModel  `json:"-"` // 元モデル
	OriginalConfigModel *pmx.PmxModel  `json:"-"` // 元モデル(ボーン追加)
	SizingModel         *pmx.PmxModel  `json:"-"` // サイジング先モデル
	SizingConfigModel   *pmx.PmxModel  `json:"-"` // サイジング先モデル(ボーン追加)
	OutputMotion        *vmd.VmdMotion `json:"-"` // 出力結果モーション

	IsSizingLeg          bool `json:"is_sizing_leg"`           // 足補正
	IsSizingUpper        bool `json:"is_sizing_upper"`         // 上半身補正
	IsSizingShoulder     bool `json:"is_sizing_shoulder"`      // 肩補正
	IsSizingArmStance    bool `json:"is_sizing_arm_stance"`    // 腕補正
	IsSizingFingerStance bool `json:"is_sizing_finger_stance"` // 指補正
	IsSizingArmTwist     bool `json:"is_sizing_arm_twist"`     // 腕捩補正
	IsSizingWrist        bool `json:"is_sizing_wrist"`         // 手首補正
	IsSizingReduction    bool `json:"is_sizing_reduction"`     // 不要キー削除補正

	CompletedSizingLeg          bool `json:"-"` // 足補正完了フラグ
	CompletedSizingUpper        bool `json:"-"` // 上半身補正完了フラグ
	CompletedSizingShoulder     bool `json:"-"` // 肩補正完了フラグ
	CompletedSizingArmStance    bool `json:"-"` // 腕補正完了フラグ
	CompletedSizingFingerStance bool `json:"-"` // 指補正完了フラグ
	CompletedSizingArmTwist     bool `json:"-"` // 腕捩補正完了フラグ
	CompletedSizingWrist        bool `json:"-"` // 手首補正完了フラグ
	CompletedSizingReduction    bool `json:"-"` // 不要キー削除補正完了フラグ

	ShoulderWeight          int   `json:"shoulder_weight"` // 肩の比重
	CompletedShoulderWeight int   `json:"-"`               // 補正完了時の肩の比重
	DefaultShoulderWeights  []int `json:"-"`               // デフォルトの肩の比重(左右別)

	originalCenterBone, originalGrooveBone, originalTrunkRootBone, originalLowerBone,
	originalUpperRootBone, originalUpperBone, originalUpper2Bone, originalNeckRootBone,
	originalLegCenterBone, originalLeftLegIkParentBone, originalLeftLegIkBone,
	originalLeftLegBone, originalLeftKneeBone, originalLeftAnkleBone,
	originalLeftToeIkBone, originalLeftToeTailBone, originalLeftHeelBone, originalLeftToePBone,
	originalLeftToeTailDBone, originalLeftHeelDBone, originalLeftToePDBone,
	originalRightLegIkParentBone, originalRightLegIkBone,
	originalRightLegBone, originalRightKneeBone, originalRightAnkleBone,
	originalRightToeIkBone, originalRightToeTailBone, originalRightHeelBone, originalRightToePBone,
	originalRightToeTailDBone, originalRightHeelDBone, originalRightToePDBone,
	originalLeftShoulderBone, originalLeftArmBone, originalLeftElbowBone, originalLeftWristBone,
	originalRightShoulderBone, originalRightArmBone, originalRightElbowBone, originalRightWristBone,
	originalLeftWristTailBone, originalRightWristTailBone *pmx.Bone // 元モデルのボーン情報

	sizingCenterBone, sizingGrooveBone, sizingTrunkRootBone, sizingLowerBone,
	sizingUpperRootBone, sizingUpperBone, sizingUpper2Bone, sizingNeckRootBone,
	sizingLegCenterBone, sizingLeftLegIkParentBone, sizingLeftLegIkBone,
	sizingLeftLegBone, sizingLeftKneeBone, sizingLeftAnkleBone,
	sizingLeftToeIkBone, sizingLeftToeTailBone, sizingLeftHeelBone, sizingLeftToePBone,
	sizingLeftToeTailDBone, sizingLeftHeelDBone, sizingLeftToePDBone,
	sizingRightLegIkParentBone, sizingRightLegIkBone, sizingRightLegBone, sizingRightKneeBone, sizingRightAnkleBone,
	sizingRightToeIkBone, sizingRightToeTailBone, sizingRightHeelBone, sizingRightToePBone,
	sizingRightToeTailDBone, sizingRightHeelDBone, sizingRightToePDBone,
	sizingLeftShoulderBone, sizingLeftArmBone, sizingLeftElbowBone, sizingLeftWristBone,
	sizingRightShoulderBone, sizingRightArmBone, sizingRightElbowBone, sizingRightWristBone,
	sizingLeftWristTailBone, sizingRightWristTailBone *pmx.Bone // サイジング先モデルのボーン情報

	sizingUpperVanillaBone, sizingUpper2VanillaBone, sizingNeckRootVanillaBone, sizingGrooveVanillaBone *pmx.Bone // サイジング先モデル(バニラ)のボーン情報
}

func NewSizingSet(index int) *SizingSet {
	return &SizingSet{
		Index: index,
	}
}

func (ss *SizingSet) CreateOutputModelPath() string {
	if ss.SizingModelPath == "" {
		return ""
	}
	// サイジング先モデルが指定されている場合、ファイル名を含める
	_, fileName, _ := mfile.SplitPath(ss.SizingModelPath)

	return mfile.CreateOutputPath(ss.SizingModelPath, fileName)
}

func (ss *SizingSet) CreateOutputMotionPath() string {
	if ss.OriginalMotionPath == "" || ss.SizingModelPath == "" {
		return ""
	}

	// サイジング先モデルが指定されている場合、ファイル名を含める
	_, fileName, _ := mfile.SplitPath(ss.SizingModelPath)

	suffix := ""
	if ss.IsSizingLeg {
		suffix += "L"
	}
	if ss.IsSizingUpper {
		suffix += "U"
	}
	if ss.IsSizingShoulder {
		suffix += "S"
	}
	if ss.IsSizingArmStance {
		suffix += "A"
	}
	if ss.IsSizingFingerStance {
		suffix += "F"
	}
	if ss.IsSizingArmTwist {
		suffix += "W"
	}
	if ss.IsSizingReduction {
		suffix += "R"
	}
	if ss.IsSizingWrist {
		suffix += "P"
	}
	if len(suffix) > 0 {
		suffix = fmt.Sprintf("_%s", suffix)
	}

	return mfile.CreateOutputPath(
		ss.OriginalMotionPath, fmt.Sprintf("%s%s", fileName, suffix))
}

func (ss *SizingSet) GetProcessCount() (processCount int) {
	if ss.OriginalConfigModel == nil || ss.SizingConfigModel == nil ||
		ss.OutputMotion == nil {
		return 0
	}

	if ss.IsSizingLeg && !ss.CompletedSizingLeg {
		processCount += 14
	}

	if ss.IsSizingUpper && !ss.CompletedSizingUpper {
		processCount += 6
	}

	if ss.IsSizingShoulder && !ss.CompletedSizingShoulder {
		processCount += 5
	}

	if ss.IsSizingArmStance && !ss.CompletedSizingArmStance {
		processCount += 4
	}

	if ss.IsSizingFingerStance && !ss.CompletedSizingFingerStance {
		processCount += 4
	}

	if ss.IsSizingWrist && !ss.CompletedSizingWrist {
		processCount += 4
	}

	if ss.IsSizingArmTwist && !ss.CompletedSizingArmTwist {
		processCount += 0
	}

	return processCount
}

func (ss *SizingSet) setMotion(originalMotion, outputMotion *vmd.VmdMotion) {
	if originalMotion == nil || outputMotion == nil {
		ss.OriginalMotionPath = ""
		ss.OriginalMotionName = ""
		ss.OriginalMotion = nil

		ss.OutputMotionPath = ""
		ss.OutputMotion = nil

		return
	}

	ss.OriginalMotionPath = originalMotion.Path()
	ss.OriginalMotionName = originalMotion.Name()
	ss.OriginalMotion = originalMotion

	ss.OutputMotionPath = outputMotion.Path()
	ss.OutputMotion = outputMotion

}

func (ss *SizingSet) setOriginalModel(originalModel, originalConfigModel *pmx.PmxModel) {
	if originalModel == nil {
		ss.OriginalModelPath = ""
		ss.OriginalModelName = ""
		ss.OriginalModel = nil
		ss.OriginalConfigModel = nil
		return
	}

	ss.OriginalModelPath = originalModel.Path()
	ss.OriginalModelName = originalModel.Name()
	ss.OriginalModel = originalModel
	ss.OriginalConfigModel = originalConfigModel
}

func (ss *SizingSet) setSizingModel(sizingModel, sizingConfigModel *pmx.PmxModel) {
	if sizingModel == nil || sizingConfigModel == nil {
		ss.SizingModelPath = ""
		ss.OutputModelName = ""
		ss.SizingConfigModel = nil
		ss.SizingModel = nil
		return
	}

	ss.SizingModelPath = sizingModel.Path()
	ss.OutputModelName = sizingModel.Name()
	ss.SizingModel = sizingModel
	ss.SizingConfigModel = sizingConfigModel
}

// LoadOriginalModel サイジング元モデルを読み込む
// TODO json の場合はフィッティングあり
func (ss *SizingSet) LoadOriginalModel(path string) {
	if path == "" {
		ss.setOriginalModel(nil, nil)
		return
	}

	var wg sync.WaitGroup
	var originalModel, originalConfigModel *pmx.PmxModel

	wg.Add(1)
	go func() {
		defer wg.Done()

		pmxRep := repository.NewPmxRepository(true)
		if data, err := pmxRep.Load(path); err == nil {
			originalModel = data.(*pmx.PmxModel)

			if err := originalModel.Bones.InsertShortageOverrideBones(); err != nil {
				mlog.ET(mi18n.T("システム用ボーン追加失敗"), err.Error())
			}
		} else {
			mlog.ET(mi18n.T("読み込み失敗"), err.Error())
		}
	}()

	wg.Add(1)
	go func() {
		defer wg.Done()

		pmxRep := repository.NewPmxRepository(false)
		if data, err := pmxRep.Load(path); err == nil {
			originalConfigModel = data.(*pmx.PmxModel)
			if err := pmxRep.CreateSticky(
				originalConfigModel,
				ss.insertShortageConfigBones,
				nil); err != nil {
				mlog.ET(mi18n.T("システム用ボーン追加失敗"), err.Error())
			}
		} else {
			mlog.ET(mi18n.T("読み込み失敗"), err.Error())
		}
	}()

	wg.Wait()

	if originalModel == nil || originalConfigModel == nil {
		ss.setOriginalModel(nil, nil)
		return
	}

	// 元モデル設定
	ss.setOriginalModel(originalModel, originalConfigModel)

	// 出力パスを設定
	ss.OutputModelPath = ss.CreateOutputModelPath()
}

// LoadSizingModel サイジング先モデルを読み込む
func (ss *SizingSet) LoadSizingModel(path string) {
	if path == "" {
		ss.setSizingModel(nil, nil)
		return
	}

	var wg sync.WaitGroup
	var sizingModel, sizingConfigModel *pmx.PmxModel

	wg.Add(1)
	go func() {
		defer wg.Done()

		pmxRep := repository.NewPmxRepository(true)
		if data, err := pmxRep.Load(path); err == nil {
			sizingModel = data.(*pmx.PmxModel)
			if err := sizingModel.Bones.InsertShortageOverrideBones(); err != nil {
				mlog.ET(mi18n.T("システム用ボーン追加失敗"), err.Error())
			}
		} else {
			mlog.ET(mi18n.T("読み込み失敗"), err.Error())
		}
	}()

	wg.Add(1)
	go func() {
		defer wg.Done()

		pmxRep := repository.NewPmxRepository(false)
		if data, err := pmxRep.Load(path); err == nil {
			sizingConfigModel = data.(*pmx.PmxModel)
			var insertDebugBones func(bones *pmx.Bones, displaySlots *pmx.DisplaySlots) error
			if mlog.IsDebug() {
				insertDebugBones = ss.insertDebugBones
			}

			if err := pmxRep.CreateSticky(
				sizingConfigModel,
				ss.insertShortageConfigBones,
				insertDebugBones); err != nil {
				mlog.ET(mi18n.T("システム用ボーン追加失敗"), err.Error())
			} else {
				if mlog.IsDebug() {
					// デバッグモードの時は、サイジング用モデルを出力
					rep := repository.NewPmxRepository(false)
					rep.Save(mfile.CreateOutputPath(path, "Sizing"), sizingConfigModel, true)
				}
			}
		} else {
			mlog.ET(mi18n.T("読み込み失敗"), err.Error())
		}
	}()

	wg.Wait()

	if sizingModel == nil || sizingConfigModel == nil {
		ss.setSizingModel(nil, nil)
		return
	}

	// サイジングモデル設定
	ss.setSizingModel(sizingModel, sizingConfigModel)

	// 出力パスを設定
	ss.OutputModelPath = ss.CreateOutputModelPath()
	ss.OutputMotionPath = ss.CreateOutputMotionPath()

	// 肩の比重を計算する
	ss.ShoulderWeight = ss.calculateShoulderWeight()
}

func (ss *SizingSet) calculateShoulderWeight() int {
	if ss.SizingModel == nil {
		return 0
	}

	// 肩の比重を計算する
	neckRootBone, err := ss.SizingModel.Bones.GetNeckRoot()
	if err != nil {
		return 0
	}

	ss.DefaultShoulderWeights = make([]int, 2)

	for i, direction := range []pmx.BoneDirection{pmx.BONE_DIRECTION_LEFT, pmx.BONE_DIRECTION_RIGHT} {
		armBone, err := ss.SizingModel.Bones.GetArm(direction)
		if err != nil {
			return 0
		}

		elbowBone, err := ss.SizingModel.Bones.GetElbow(direction)
		if err != nil {
			return 0
		}

		wristBone, err := ss.SizingModel.Bones.GetWrist(direction)
		if err != nil {
			return 0
		}

		shoulderLength := neckRootBone.Position.Distance(armBone.Position)
		armLength := armBone.Position.Distance(elbowBone.Position) + elbowBone.Position.Distance(wristBone.Position)

		ss.DefaultShoulderWeights[i] = int(shoulderLength / armLength * 100)
	}

	return (ss.DefaultShoulderWeights[0] + ss.DefaultShoulderWeights[1]) / 2
}

func (ss *SizingSet) insertDebugBones(bones *pmx.Bones, displaySlots *pmx.DisplaySlots) error {
	rootBone, _ := bones.GetRoot()
	centerBone, _ := bones.GetCenter()
	rightLegIkBone, _ := bones.GetLegIk(pmx.BONE_DIRECTION_RIGHT)
	leftLegIkBone, _ := bones.GetLegIk(pmx.BONE_DIRECTION_LEFT)

	for _, v := range [][]any{
		{"先足中心", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"先理想足中心", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"元重心", rootBone.Index(), mmath.NewMVec3(), "足04"},
		{"先重心", rootBone.Index(), mmath.NewMVec3(), "足04"},
		{"理想重心", rootBone.Index(), mmath.NewMVec3(), "足04"},
		{"重心センター", centerBone.ParentIndex, centerBone.Position, "足04"},
		{"左足IK補正前", rootBone.Index(), mmath.NewMVec3(), "足06_Y補正"},
		{"左足IK理想", rootBone.Index(), mmath.NewMVec3(), "足06_Y補正"},
		{"左IK親初期", rootBone.Index(), mmath.NewMVec3(), "足06_Y補正"},
		{"左足IK初期", rootBone.Index(), mmath.NewMVec3(), "足06_Y補正"},
		{"左足IK補正後", leftLegIkBone.ParentIndex, leftLegIkBone.Position, "足06_Y補正"},
		{"右足IK補正前", rootBone.Index(), mmath.NewMVec3(), "足06_Y補正"},
		{"右足IK理想", rootBone.Index(), mmath.NewMVec3(), "足06_Y補正"},
		{"右IK親初期", rootBone.Index(), mmath.NewMVec3(), "足06_Y補正"},
		{"右足IK初期", rootBone.Index(), mmath.NewMVec3(), "足06_Y補正"},
		{"右足IK補正後", rightLegIkBone.ParentIndex, rightLegIkBone.Position, "足06_Y補正"},
		{"左つま先補正前", rootBone.Index(), mmath.NewMVec3(), "足06_つま先"},
		{"左つま先理想", rootBone.Index(), mmath.NewMVec3(), "足06_つま先"},
		{"右つま先補正前", rootBone.Index(), mmath.NewMVec3(), "足06_つま先"},
		{"右つま先理想", rootBone.Index(), mmath.NewMVec3(), "足06_つま先"},
		{"左かかと補正前", rootBone.Index(), mmath.NewMVec3(), "足06_かかと"},
		{"左かかと理想", rootBone.Index(), mmath.NewMVec3(), "足06_かかと"},
		{"右かかと補正前", rootBone.Index(), mmath.NewMVec3(), "足06_かかと"},
		{"右かかと理想", rootBone.Index(), mmath.NewMVec3(), "足06_かかと"},
		{"左足継承", leftLegIkBone.ParentIndex, leftLegIkBone.Position, "足08"},
		{"右足継承", rightLegIkBone.ParentIndex, rightLegIkBone.Position, "足08"},
		{"上半身Root", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"上半身Tgt", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"上半身IK", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"先左腕", rootBone.Index(), mmath.NewMVec3(), "肩02"},
		{"先左腕比率", rootBone.Index(), mmath.NewMVec3(), "肩02"},
		{"先左腕Y固定", rootBone.Index(), mmath.NewMVec3(), "肩02"},
		{"先左腕理想", rootBone.Index(), mmath.NewMVec3(), "肩02"},
		{"先左肩結果", rootBone.Index(), mmath.NewMVec3(), "肩02"},
		{"先右腕", rootBone.Index(), mmath.NewMVec3(), "肩02"},
		{"先右腕比率", rootBone.Index(), mmath.NewMVec3(), "肩02"},
		{"先右腕Y固定", rootBone.Index(), mmath.NewMVec3(), "肩02"},
		{"先右腕理想", rootBone.Index(), mmath.NewMVec3(), "肩02"},
		{"先右肩結果", rootBone.Index(), mmath.NewMVec3(), "肩02"},
		{"位元左手首", rootBone.Index(), mmath.NewMVec3(), "位置02_元"},
		{"位元右手首", rootBone.Index(), mmath.NewMVec3(), "位置02_元"},
		{"位先左手首", rootBone.Index(), mmath.NewMVec3(), "位置02_先"},
		{"位先左手首首理", rootBone.Index(), mmath.NewMVec3(), "位置02_先"},
		{"位先左手首腹理", rootBone.Index(), mmath.NewMVec3(), "位置02_先"},
		{"位先左手首理想", rootBone.Index(), mmath.NewMVec3(), "位置02_先"},
		{"位先右手首", rootBone.Index(), mmath.NewMVec3(), "位置02_先"},
		{"位先右手首首理", rootBone.Index(), mmath.NewMVec3(), "位置02_先"},
		{"位先右手首腹理", rootBone.Index(), mmath.NewMVec3(), "位置02_先"},
		{"位先右手首理想", rootBone.Index(), mmath.NewMVec3(), "位置02_先"},
	} {
		boneName := v[0].(string)
		parentBoneIndex := v[1].(int)
		position := v[2].(*mmath.MVec3)
		displaySlotName := v[3].(string)

		bone := pmx.NewBone()
		bone.SetName(boneName)
		bone.SetEnglishName(boneName)
		bone.BoneFlag = pmx.BONE_FLAG_IS_VISIBLE | pmx.BONE_FLAG_CAN_MANIPULATE | pmx.BONE_FLAG_CAN_ROTATE | pmx.BONE_FLAG_CAN_TRANSLATE
		bone.Position = position
		bone.IsSystem = true

		if parentBone, _ := bones.Get(parentBoneIndex); parentBone != nil {
			bone.ParentIndex = parentBone.Index()
		}

		if err := bones.Insert(bone); err != nil {
			return err
		}

		if !displaySlots.ContainsByName(displaySlotName) {
			displaySlot := pmx.NewDisplaySlot()
			displaySlot.SetName(displaySlotName)
			displaySlot.SetEnglishName(displaySlotName)
			displaySlots.Append(displaySlot)
		}

		displaySlot, _ := displaySlots.GetByName(displaySlotName)
		if displaySlot != nil {
			displaySlot.References = append(displaySlot.References,
				pmx.NewDisplaySlotReferenceByValues(pmx.DISPLAY_TYPE_BONE, bone.Index()))
		}

		bones.Setup()
	}

	return nil
}

// InsertShortageSizingConfigBones サイジング用不足ボーン作成
func (ss *SizingSet) insertShortageConfigBones(vertices *pmx.Vertices, bones *pmx.Bones) error {

	// 体幹系
	for _, funcs := range [][]func() (*pmx.Bone, error){
		{bones.GetRoot, bones.CreateRoot},
		{bones.GetTrunkRoot, bones.CreateTrunkRoot},
		{bones.GetLowerRoot, bones.CreateLowerRoot},
		{bones.GetLegCenter, bones.CreateLegCenter},
		{bones.GetUpperRoot, bones.CreateUpperRoot},
		{bones.GetNeckRoot, bones.CreateNeckRoot},
		{bones.GetHeadTail, bones.CreateHeadTail},
	} {
		getFunc := funcs[0]
		createFunc := funcs[1]

		if bone, err := getFunc(); err != nil && err == merr.NameNotFoundError && bone == nil {
			if bone, err := createFunc(); err == nil && bone != nil {
				bone.IsSystem = true

				if err := bones.Insert(bone); err != nil {
					return err
				} else {
					// 追加したボーンの親ボーンを、同じく親ボーンに設定しているボーンの親ボーンを追加ボーンに置き換える
					bones.ForEach(func(i int, b *pmx.Bone) bool {
						if b.ParentIndex == bone.ParentIndex && b.Index() != bone.Index() &&
							b.EffectIndex != bone.Index() && bone.EffectIndex != b.Index() &&
							((strings.Contains(bone.Name(), "上") && !strings.Contains(b.Name(), "下") &&
								!strings.Contains(b.Name(), "左") && !strings.Contains(b.Name(), "右")) ||
								(strings.Contains(bone.Name(), "下") && !strings.Contains(b.Name(), "上") &&
									!strings.Contains(b.Name(), "左") && !strings.Contains(b.Name(), "右")) ||
								(bone.Name() == pmx.NECK_ROOT.String() || bone.Name() == pmx.LEG_CENTER.String())) {
							b.ParentIndex = bone.Index()
							return true
						}
						return true
					})
					// セットアップしなおし
					bones.Setup()
				}
			} else {
				return err
			}
		} else if err != nil {
			return err
		} else {
			switch bone.Name() {
			case pmx.NECK.String():
				if neckRoot, err := bones.GetNeckRoot(); err == nil {
					bone.ParentIndex = neckRoot.Index()
				}
			}
		}
	}

	// 左右系
	for _, direction := range []pmx.BoneDirection{pmx.BONE_DIRECTION_LEFT, pmx.BONE_DIRECTION_RIGHT} {
		for _, funcs := range [][]func(direction pmx.BoneDirection) (*pmx.Bone, error){
			{bones.GetShoulderRoot, bones.CreateShoulderRoot},
			{bones.GetWristTail, bones.CreateWristTail},
			{bones.GetThumbTail, bones.CreateThumbTail},
			{bones.GetIndexTail, bones.CreateIndexTail},
			{bones.GetMiddleTail, bones.CreateMiddleTail},
			{bones.GetRingTail, bones.CreateRingTail},
			{bones.GetPinkyTail, bones.CreatePinkyTail},
			{bones.GetLegRoot, bones.CreateLegRoot},
			{bones.GetToeT, bones.CreateToeT},
			{bones.GetToeP, bones.CreateToeP},
			{bones.GetToeC, bones.CreateToeC},
			{bones.GetLegD, bones.CreateLegD},
			{bones.GetKneeD, bones.CreateKneeD},
			{bones.GetAnkleD, bones.CreateAnkleD},
			{bones.GetToeEx, bones.CreateToeEx},
			{bones.GetToeTD, bones.CreateToeTD},
			{bones.GetToePD, bones.CreateToePD},
			{bones.GetToeCD, bones.CreateToeCD},
			{bones.GetHeel, bones.CreateHeel},
			{bones.GetHeelD, bones.CreateHeelD},
		} {
			getFunc := funcs[0]
			createFunc := funcs[1]

			if bone, err := getFunc(direction); err != nil && err == merr.NameNotFoundError && bone == nil {
				if bone, err := createFunc(direction); err == nil && bone != nil {
					bone.IsSystem = true

					if bone.Name() == pmx.TOE_T.StringFromDirection(direction) {
						// つま先の位置は、足首・足首D・足先EXの中で最もZ値が小さい位置にする
						vertexMap := vertices.GetMapByBoneIndex(1e-1)
						if vertexMap != nil {
							for _, ankleBoneName := range []string{
								pmx.ANKLE.StringFromDirection(direction),
								pmx.ANKLE_D.StringFromDirection(direction),
								pmx.TOE_EX.StringFromDirection(direction),
							} {
								ankleBone, _ := bones.GetByName(ankleBoneName)
								if ankleBone == nil {
									continue
								}
								if boneVertices, ok := vertexMap[ankleBone.Index()]; ok && boneVertices != nil {
									for _, vertex := range boneVertices {
										if vertex.Position.Z < bone.Position.Z && vertex.Position.Y < bone.Position.Y {
											bone.Position = vertex.Position.Copy()
										}
									}
								}
							}
							bone.Position.Y = 0
						}
					}

					if err := bones.Insert(bone); err != nil {
						return err
					} else {
						// 追加したボーンの親ボーンを、同じく親ボーンに設定しているボーンの親ボーンを追加ボーンに置き換える
						bones.ForEach(func(i int, b *pmx.Bone) bool {
							if b.ParentIndex == bone.ParentIndex && b.Index() != bone.Index() &&
								b.EffectIndex != bone.Index() && bone.EffectIndex != b.Index() &&
								((strings.Contains(bone.Name(), "左") && strings.Contains(b.Name(), "左")) ||
									(strings.Contains(bone.Name(), "右") && strings.Contains(b.Name(), "右"))) {
								b.ParentIndex = bone.Index()
								return false
							}
							return true
						})
						// セットアップしなおし
						bones.Setup()
					}
				} else {
					return err
				}
			} else if err != nil {
				return err
			}
		}

		{
			// 親指0
			if bone, err := bones.GetThumb(direction, 0); err != nil && err == merr.NameNotFoundError && bone == nil {
				if bone, err := bones.CreateThumb0(direction); err == nil && bone != nil {
					if err := bones.Insert(bone); err != nil {
						return err
					}
					bones.Setup()
				} else {
					return err
				}
			} else if err != nil {
				return err
			}
		}
	}

	return nil
}

// LoadMotion サイジング対象モーションを読み込む
func (ss *SizingSet) LoadMotion(path string) {
	if path == "" {
		ss.setMotion(nil, nil)
		return
	}

	var wg sync.WaitGroup
	var originalMotion, sizingMotion *vmd.VmdMotion

	wg.Add(1)
	go func() {
		defer wg.Done()

		vmdRep := repository.NewVmdVpdRepository(false)
		if data, err := vmdRep.Load(path); err == nil {
			originalMotion = data.(*vmd.VmdMotion)
		} else {
			mlog.ET(mi18n.T("読み込み失敗"), err.Error())
		}
	}()

	wg.Add(1)
	go func() {
		defer wg.Done()

		vmdRep := repository.NewVmdVpdRepository(true)
		if data, err := vmdRep.Load(path); err == nil {
			sizingMotion = data.(*vmd.VmdMotion)
		} else {
			mlog.ET(mi18n.T("読み込み失敗"), err.Error())
		}
	}()

	wg.Wait()

	ss.setMotion(originalMotion, sizingMotion)

	outputPath := ss.CreateOutputMotionPath()
	ss.OutputMotionPath = outputPath
}

func (ss *SizingSet) Delete() {
	ss.OriginalMotionPath = ""
	ss.OriginalModelPath = ""
	ss.SizingModelPath = ""
	ss.OutputMotionPath = ""
	ss.OutputModelPath = ""

	ss.OriginalMotionName = ""
	ss.OriginalModelName = ""
	ss.OutputModelName = ""

	ss.OriginalMotion = nil
	ss.OriginalModel = nil
	ss.OriginalConfigModel = nil
	ss.SizingModel = nil
	ss.SizingConfigModel = nil
	ss.OutputMotion = nil

	ss.IsSizingLeg = false
	ss.IsSizingUpper = false
	ss.IsSizingShoulder = false
	ss.IsSizingArmStance = false
	ss.IsSizingFingerStance = false
	ss.IsSizingArmTwist = false
	ss.IsSizingWrist = false
	ss.IsSizingReduction = false

	ss.CompletedSizingLeg = false
	ss.CompletedSizingUpper = false
	ss.CompletedSizingShoulder = false
	ss.CompletedSizingArmStance = false
	ss.CompletedSizingFingerStance = false
	ss.CompletedSizingArmTwist = false
	ss.CompletedSizingReduction = false
}
