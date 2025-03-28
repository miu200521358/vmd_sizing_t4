package domain

import (
	"fmt"

	"github.com/miu200521358/mlib_go/pkg/domain/pmx"
	"github.com/miu200521358/mlib_go/pkg/domain/vmd"
	"github.com/miu200521358/mlib_go/pkg/infrastructure/mfile"
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

	OriginalMotion    *vmd.VmdMotion `json:"-"` // 元モデル
	OriginalModel     *pmx.PmxModel  `json:"-"` // 元モーション
	SizingModel       *pmx.PmxModel  `json:"-"` // サイジング先モデル
	SizingConfigModel *pmx.PmxModel  `json:"-"` // サイジング先モデル(ボーン追加)
	OutputMotion      *vmd.VmdMotion `json:"-"` // 出力結果モーション

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

func (ss *SizingSet) SetOriginalMotion(motion *vmd.VmdMotion) {
	if motion == nil {
		ss.OriginalMotionPath = ""
		ss.OriginalMotionName = ""
		ss.OriginalMotion = nil
		return
	}

	ss.OriginalMotionPath = motion.Path()
	ss.OriginalMotionName = motion.Name()
	ss.OriginalMotion = motion
}

func (ss *SizingSet) SetOriginalModel(model *pmx.PmxModel) {
	if model == nil {
		ss.OriginalModelPath = ""
		ss.OriginalModelName = ""
		ss.OriginalModel = nil
		return
	}

	ss.OriginalModelPath = model.Path()
	ss.OriginalModelName = model.Name()
	ss.OriginalModel = model
}

func (ss *SizingSet) SetOutputMotion(motion *vmd.VmdMotion) {
	if motion == nil {
		ss.OutputMotionPath = ""
		ss.OutputMotion = nil
		return
	}

	ss.OutputMotionPath = motion.Path()
	ss.OutputMotion = motion
}

func (ss *SizingSet) SetSizingModel(model *pmx.PmxModel) {
	if model == nil {
		ss.SizingModelPath = ""
		ss.OutputModelName = ""
		ss.SizingModel = nil
		return
	}

	ss.SizingModelPath = model.Path()
	ss.OutputModelName = model.Name()
	ss.SizingModel = model
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
