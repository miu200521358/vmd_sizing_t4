package domain

import (
	"fmt"
	"sync"

	"github.com/miu200521358/mlib_go/pkg/config/mi18n"
	"github.com/miu200521358/mlib_go/pkg/config/mlog"
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
	IsSizingReduction    bool `json:"is_sizing_reduction"`     // 不要キー削除補正

	CompletedSizingLeg          bool `json:"-"` // 足補正完了フラグ
	CompletedSizingUpper        bool `json:"-"` // 上半身補正完了フラグ
	CompletedSizingShoulder     bool `json:"-"` // 肩補正完了フラグ
	CompletedSizingArmStance    bool `json:"-"` // 腕補正完了フラグ
	CompletedSizingFingerStance bool `json:"-"` // 指補正完了フラグ
	CompletedSizingArmTwist     bool `json:"-"` // 腕捩補正完了フラグ
	CompletedSizingReduction    bool `json:"-"` // 不要キー削除補正完了フラグ
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
	if ss.OriginalMotionPath == "" {
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
	if len(suffix) > 0 {
		suffix = fmt.Sprintf("_%s", suffix)
	}

	return mfile.CreateOutputPath(
		ss.OriginalMotionPath, fmt.Sprintf("%s%s", fileName, suffix))
}

func (ss *SizingSet) GetProcessCount() (processCount, completedCount int) {
	if ss.OriginalConfigModel == nil || ss.SizingConfigModel == nil ||
		ss.OutputMotion == nil {
		return 0, 0
	}

	if ss.IsSizingLeg {
		processCount += 12
		if ss.CompletedSizingLeg {
			completedCount += 12
		}
	}
	if ss.IsSizingUpper {
		processCount += 0
		if ss.CompletedSizingUpper {
			completedCount += 0
		}
	}
	if ss.IsSizingShoulder {
		processCount += 0
		if ss.CompletedSizingShoulder {
			completedCount += 0
		}
	}
	if ss.IsSizingArmStance {
		processCount += 0
		if ss.CompletedSizingArmStance {
			completedCount += 0
		}
	}
	if ss.IsSizingFingerStance {
		processCount += 0
		if ss.CompletedSizingFingerStance {
			completedCount += 0
		}
	}
	if ss.IsSizingArmTwist {
		processCount += 0
		if ss.CompletedSizingArmTwist {
			completedCount += 0
		}
	}
	if ss.IsSizingReduction {
		processCount += 0
		if ss.CompletedSizingReduction {
			completedCount += 0
		}
	}

	return processCount, completedCount
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
		} else {
			mlog.ET(mi18n.T("読み込み失敗"), err.Error())
		}
	}()

	wg.Wait()

	if originalModel == nil || originalConfigModel == nil {
		ss.setOriginalModel(nil, nil)
		return
	}

	if err := originalModel.Bones.InsertShortageOverrideBones(); err != nil {
		mlog.ET(mi18n.T("システム用ボーン追加失敗"), err.Error())
		ss.setOriginalModel(nil, nil)
		return
	}

	if err := originalConfigModel.Bones.InsertShortageConfigBones(); err != nil {
		mlog.ET(mi18n.T("システム用ボーン追加失敗"), err.Error())
		ss.setOriginalModel(nil, nil)
		return
	}

	// 元モデル設定
	ss.setOriginalModel(originalModel, originalConfigModel)

	// 出力パスを設定
	outputPath := ss.CreateOutputModelPath()
	ss.OutputModelPath = outputPath

	if mlog.IsVerbose() {
		pmxRep := repository.NewPmxRepository(true)
		if err := pmxRep.Save(outputPath, originalConfigModel, true); err != nil {
			mlog.ET(mi18n.T("保存失敗"), err.Error())
		}
	}
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
		} else {
			mlog.ET(mi18n.T("読み込み失敗"), err.Error())
		}
	}()

	wg.Wait()

	if sizingModel == nil || sizingConfigModel == nil {
		ss.setSizingModel(nil, nil)
		return
	}

	if err := sizingModel.Bones.InsertShortageOverrideBones(); err != nil {
		mlog.ET(mi18n.T("システム用ボーン追加失敗"), err.Error())
		ss.setOriginalModel(nil, nil)
		return
	}

	if err := sizingConfigModel.Bones.InsertShortageConfigBones(); err != nil {
		mlog.ET(mi18n.T("システム用ボーン追加失敗"), err.Error())
		ss.setSizingModel(nil, nil)
		return
	}

	// サイジングモデル設定
	ss.setSizingModel(sizingModel, sizingConfigModel)

	// 出力パスを設定
	outputPath := ss.CreateOutputModelPath()
	ss.OutputModelPath = outputPath

	if mlog.IsVerbose() {
		pmxRep := repository.NewPmxRepository(true)
		if err := pmxRep.Save(outputPath, sizingConfigModel, true); err != nil {
			mlog.ET(mi18n.T("保存失敗"), err.Error())
		}
	}
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
			for boneName := range pmx.GetStandardBoneConfigs() {
				// 枠だけ用意しておく
				sizingMotion.BoneFrames.Get(boneName.String())
				sizingMotion.BoneFrames.Get(boneName.Left())
				sizingMotion.BoneFrames.Get(boneName.Right())
			}
		} else {
			mlog.ET(mi18n.T("読み込み失敗"), err.Error())
		}
	}()

	wg.Wait()

	ss.setMotion(originalMotion, sizingMotion)

	outputPath := ss.CreateOutputMotionPath()
	ss.OutputMotionPath = outputPath
}

// PrepareBoneNameFrames モーションのボーンフレームを準備する(並列対策)
func (ss *SizingSet) PrepareBoneNameFrames() {
	if ss.OriginalMotion != nil && ss.OriginalConfigModel != nil {
		ss.OriginalConfigModel.Bones.ForEach(func(index int, bone *pmx.Bone) {
			ss.OriginalMotion.BoneFrames.Get(bone.Name())
		})
	}

	if ss.OutputMotion != nil && ss.SizingConfigModel != nil {
		ss.SizingConfigModel.Bones.ForEach(func(index int, bone *pmx.Bone) {
			ss.OutputMotion.BoneFrames.Get(bone.Name())
		})
	}
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
	ss.IsSizingReduction = false

	ss.CompletedSizingLeg = false
	ss.CompletedSizingUpper = false
	ss.CompletedSizingShoulder = false
	ss.CompletedSizingArmStance = false
	ss.CompletedSizingFingerStance = false
	ss.CompletedSizingArmTwist = false
	ss.CompletedSizingReduction = false
}
