package domain

import (
	"fmt"
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
	// OriginalGravityVolumes  map[string]float64 `json:"-"`               // 元モデルの重心体積
	// SizingGravityVolumes    map[string]float64 `json:"-"`               // サイジング先モデルの重心体積

	originalBoneCache      map[string]*pmx.Bone // 元モデルのボーンキャッシュ
	sizingBoneCache        map[string]*pmx.Bone // サイジング先モデルのボーンキャッシュ
	sizingVanillaBoneCache map[string]*pmx.Bone // サイジング先モデル(バニラ)のボーンキャッシュ
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
	if ss.IsSizingArmStance {
		suffix += "A"
	}
	if ss.IsSizingLeg {
		suffix += "L"
	}
	if ss.IsSizingUpper {
		suffix += "U"
	}
	if ss.IsSizingShoulder {
		suffix += "S"
	}
	if ss.IsSizingFingerStance {
		suffix += "F"
	}
	if ss.IsSizingArmTwist {
		suffix += "W"
	}
	if ss.IsSizingWrist {
		suffix += "P"
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

func (ss *SizingSet) GetProcessCount() (processCount int) {
	if ss.OriginalConfigModel == nil || ss.SizingConfigModel == nil ||
		ss.OutputMotion == nil {
		return 0
	}

	maxFrame := int(ss.OutputMotion.MaxFrame())

	if ss.IsSizingLeg && !ss.CompletedSizingLeg {
		// 8: computeVmdDeltas
		// 1: FK焼き込み
		// 4*2: calculate系 / update系
		// 3*11: updateOutputMotion (interval / full)
		// 1: updateLegIkOffset
		processCount += maxFrame * (8 + 1 + 4*2 + 3*11 + 1)
	}

	if ss.IsSizingUpper && !ss.CompletedSizingUpper {
		// 2: computeVmdDeltas
		// 2: computeMorphVmdDeltas
		// 1*2: calculate系 / update系
		// 3*2: updateOutputMotion (active / interval / full)
		processCount += maxFrame * (2 + 2 + 1*2 + 3*2)
	}

	if ss.IsSizingShoulder && !ss.CompletedSizingShoulder {
		processCount += 3 + maxFrame*2*2
	}

	if ss.IsSizingArmStance && !ss.CompletedSizingArmStance {
		processCount += 3
	}

	if ss.IsSizingFingerStance && !ss.CompletedSizingFingerStance {
		processCount += 3
	}

	if ss.IsSizingWrist && !ss.CompletedSizingWrist {
		processCount += 0
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
func (ss *SizingSet) LoadOriginalModel(path string) error {
	ss.originalBoneCache = make(map[string]*pmx.Bone)

	if path == "" {
		ss.setOriginalModel(nil, nil)
		return nil
	}

	var wg sync.WaitGroup
	var originalModel, originalConfigModel *pmx.PmxModel

	wg.Add(1)
	errChan := make(chan error, 2)
	go func() {
		defer wg.Done()

		pmxRep := repository.NewPmxRepository(true)
		if data, err := pmxRep.Load(path); err == nil {
			originalModel = data.(*pmx.PmxModel)

			if err := originalModel.Bones.InsertShortageOverrideBones(); err != nil {
				mlog.ET(mi18n.T("システム用ボーン追加失敗"), err, "")
				errChan <- err
			}
		} else {
			mlog.ET(mi18n.T("読み込み失敗"), err, "")
			errChan <- err
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
				mlog.ET(mi18n.T("システム用ボーン追加失敗"), err, "")
				errChan <- err
			} else {
				if mlog.IsDebug() {
					// デバッグモードの時は、サイジング用モデルを出力
					rep := repository.NewPmxRepository(false)
					rep.Save(mfile.CreateOutputPath(path, "Sizing"), originalConfigModel, true)
				}
			}
		} else {
			mlog.ET(mi18n.T("読み込み失敗"), err, "")
			errChan <- err
		}
	}()

	wg.Wait()

	close(errChan)

	for err := range errChan {
		if err != nil {
			ss.setOriginalModel(nil, nil)
			return err
		}
	}

	// 元モデル設定
	ss.setOriginalModel(originalModel, originalConfigModel)

	// 肩の比重を計算する
	ss.ShoulderWeight = ss.calculateShoulderWeight()
	ss.CompletedShoulderWeight = ss.ShoulderWeight

	// 出力パスを設定
	ss.OutputModelPath = ss.CreateOutputModelPath()

	return nil
}

// LoadSizingModel サイジング先モデルを読み込む
func (ss *SizingSet) LoadSizingModel(path string) error {
	ss.sizingBoneCache = make(map[string]*pmx.Bone)
	ss.sizingVanillaBoneCache = make(map[string]*pmx.Bone)

	if path == "" {
		ss.setSizingModel(nil, nil)
		return nil
	}

	var wg sync.WaitGroup
	var sizingModel, sizingConfigModel *pmx.PmxModel

	errChan := make(chan error, 2)

	wg.Add(1)
	go func() {
		defer wg.Done()

		pmxRep := repository.NewPmxRepository(true)
		if data, err := pmxRep.Load(path); err == nil {
			sizingModel = data.(*pmx.PmxModel)
			if err := sizingModel.Bones.InsertShortageOverrideBones(); err != nil {
				mlog.ET(mi18n.T("システム用ボーン追加失敗"), err, "")
				errChan <- err
			}
		} else {
			mlog.ET(mi18n.T("読み込み失敗"), err, "")
			errChan <- err
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
				mlog.ET(mi18n.T("システム用ボーン追加失敗"), err, "")
				errChan <- err
			} else {
				if mlog.IsDebug() {
					// デバッグモードの時は、サイジング用モデルを出力
					rep := repository.NewPmxRepository(false)
					rep.Save(mfile.CreateOutputPath(path, "Sizing"), sizingConfigModel, true)
				}
			}
		} else {
			mlog.ET(mi18n.T("読み込み失敗"), err, "")
			errChan <- err
		}
	}()

	wg.Wait()
	close(errChan)

	for err := range errChan {
		if err != nil {
			ss.setSizingModel(nil, nil)
			return err
		}
	}

	// サイジングモデル設定
	ss.setSizingModel(sizingModel, sizingConfigModel)

	// 出力パスを設定
	ss.OutputModelPath = ss.CreateOutputModelPath()
	ss.OutputMotionPath = ss.CreateOutputMotionPath()

	// 肩の比重を計算する
	ss.ShoulderWeight = ss.calculateShoulderWeight()
	ss.CompletedShoulderWeight = ss.ShoulderWeight

	// // 重心体積を計算する
	// ss.OriginalGravityVolumes = calculateGravityVolume(ss.OriginalConfigModel)
	// ss.SizingGravityVolumes = calculateGravityVolume(ss.SizingConfigModel)

	return nil
}

// LoadMotion サイジング対象モーションを読み込む
func (ss *SizingSet) LoadMotion(path string) error {

	if path == "" {
		ss.setMotion(nil, nil)
		return nil
	}

	var wg sync.WaitGroup
	var originalMotion, sizingMotion *vmd.VmdMotion
	errChan := make(chan error, 2)

	wg.Add(1)
	go func() {
		defer wg.Done()

		vmdRep := repository.NewVmdVpdRepository(false)
		if data, err := vmdRep.Load(path); err == nil {
			originalMotion = data.(*vmd.VmdMotion)
		} else {
			mlog.ET(mi18n.T("読み込み失敗"), err, "")
			errChan <- err
		}
	}()

	wg.Add(1)
	go func() {
		defer wg.Done()

		vmdRep := repository.NewVmdVpdRepository(true)
		if data, err := vmdRep.Load(path); err == nil {
			sizingMotion = data.(*vmd.VmdMotion)
		} else {
			mlog.ET(mi18n.T("読み込み失敗"), err, "")
			errChan <- err
		}
	}()

	wg.Wait()
	close(errChan)

	for err := range errChan {
		if err != nil {
			return err
		}
	}

	ss.setMotion(originalMotion, sizingMotion)

	// 肩の比重を計算する
	ss.ShoulderWeight = ss.calculateShoulderWeight()
	ss.CompletedShoulderWeight = ss.ShoulderWeight

	// 出力パスを設定
	outputPath := ss.CreateOutputMotionPath()
	ss.OutputMotionPath = outputPath

	return nil
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

		shoulderLength := neckRootBone.Position.Distance(armBone.Position)
		armLength := armBone.Position.Distance(elbowBone.Position)

		ss.DefaultShoulderWeights[i] = int(shoulderLength / armLength * 100)
	}

	return (ss.DefaultShoulderWeights[0] + ss.DefaultShoulderWeights[1]) / 2
}

func (ss *SizingSet) insertDebugBones(bones *pmx.Bones, displaySlots *pmx.DisplaySlots) error {
	rootBone, _ := bones.GetRoot()

	for _, v := range [][]any{
		// 下半身補正
		{"元今下2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"元今足中2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"元今左足2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"元今右足2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"先今下2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"先結下2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"先今足中2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"先理足中2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"先結足中2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"先今左足2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"先理左足2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"先結左足2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"先今右足2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"先理右足2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"先結右足2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"元今左尻2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"先今左尻2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"先結左尻2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"元今右尻2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"先今右尻2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		{"先結右尻2", rootBone.Index(), mmath.NewMVec3(), "足02"},
		// 足IK補正
		{"元今足中4", rootBone.Index(), mmath.NewMVec3(), "足04元"},
		{"元今左膝4", rootBone.Index(), mmath.NewMVec3(), "足04元"},
		{"元今右膝4", rootBone.Index(), mmath.NewMVec3(), "足04元"},
		{"元今左足4", rootBone.Index(), mmath.NewMVec3(), "足04元"},
		{"元今右足4", rootBone.Index(), mmath.NewMVec3(), "足04元"},
		{"元今左足首4", rootBone.Index(), mmath.NewMVec3(), "足04元"},
		{"元今右足首4", rootBone.Index(), mmath.NewMVec3(), "足04元"},
		{"先今足中4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先今左足4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先今右足4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先今左膝4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先理左膝4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先結左膝4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先今右膝4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先理右膝4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先結右膝4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先今左足首4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先理左足首4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先結左足首4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先今右足首4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先理右足首4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先結右足首4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先今左足K4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先結左足K4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先今右足K4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先結右足K4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先今左爪K4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先理左爪K4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先結左爪K4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先今右爪K4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先理右爪K4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		{"先結右爪K4", rootBone.Index(), mmath.NewMVec3(), "足04先"},
		// センター補正
		{"元今体軸6", rootBone.Index(), mmath.NewMVec3(), "足06"},
		{"先今体軸6", rootBone.Index(), mmath.NewMVec3(), "足06"},
		{"先結体軸6", rootBone.Index(), mmath.NewMVec3(), "足06"},
		// 足IK補正（2回目）
		{"元今左足首8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"元今右足首8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"元今左爪先8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"元今右爪先8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"元今左踵8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"元今右踵8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先今左足首8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先理左足首8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先結左足首8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先今右足首8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先理右足首8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先結右足首8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先今左爪先8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先理左爪先8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先結左爪先8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先今右爪先8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先理右爪先8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先結右爪先8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先今左踵8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先理左踵8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先結左踵8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先今右踵8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先理右踵8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先結右踵8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先今左足K8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先理左足K8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先結左足K8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先今右足K8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先理右足K8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		{"先結右足K8", rootBone.Index(), mmath.NewMVec3(), "足08"},
		// センター補正(2回目)
		{"元今体軸0", rootBone.Index(), mmath.NewMVec3(), "足10"},
		{"先今体軸0", rootBone.Index(), mmath.NewMVec3(), "足10"},
		{"先結体軸0", rootBone.Index(), mmath.NewMVec3(), "足10"},
		{"元今左膝0", rootBone.Index(), mmath.NewMVec3(), "足10"},
		{"元今右膝0", rootBone.Index(), mmath.NewMVec3(), "足10"},
		{"先今左膝0", rootBone.Index(), mmath.NewMVec3(), "足10"},
		{"先理左膝0", rootBone.Index(), mmath.NewMVec3(), "足10"},
		{"先結左膝0", rootBone.Index(), mmath.NewMVec3(), "足10"},
		{"先今右膝0", rootBone.Index(), mmath.NewMVec3(), "足10"},
		{"先理右膝0", rootBone.Index(), mmath.NewMVec3(), "足10"},
		{"先結右膝0", rootBone.Index(), mmath.NewMVec3(), "足10"},
		// 出力
		{"元今下1", rootBone.Index(), mmath.NewMVec3(), "足11左"},
		{"先今下1", rootBone.Index(), mmath.NewMVec3(), "足11左"},
		{"元今左足1", rootBone.Index(), mmath.NewMVec3(), "足11左"},
		{"先今左足1", rootBone.Index(), mmath.NewMVec3(), "足11左"},
		{"元今左膝1", rootBone.Index(), mmath.NewMVec3(), "足11左"},
		{"先今左膝1", rootBone.Index(), mmath.NewMVec3(), "足11左"},
		{"元今左足首1", rootBone.Index(), mmath.NewMVec3(), "足11左"},
		{"先今左足首1", rootBone.Index(), mmath.NewMVec3(), "足11左"},
		{"元今左足EX1", rootBone.Index(), mmath.NewMVec3(), "足11左"},
		{"先今左足EX1", rootBone.Index(), mmath.NewMVec3(), "足11左"},
		{"元今左爪先D1", rootBone.Index(), mmath.NewMVec3(), "足11左"},
		{"先今左爪先D1", rootBone.Index(), mmath.NewMVec3(), "足11左"},
		{"元今左爪先PD1", rootBone.Index(), mmath.NewMVec3(), "足11左"},
		{"先今左爪先PD1", rootBone.Index(), mmath.NewMVec3(), "足11左"},
		{"元今左爪先CD1", rootBone.Index(), mmath.NewMVec3(), "足11左"},
		{"先今左爪先CD1", rootBone.Index(), mmath.NewMVec3(), "足11左"},
		{"元今左踵D1", rootBone.Index(), mmath.NewMVec3(), "足11左"},
		{"先今左踵D1", rootBone.Index(), mmath.NewMVec3(), "足11左"},
		{"元今右足1", rootBone.Index(), mmath.NewMVec3(), "足11右"},
		{"先今右足1", rootBone.Index(), mmath.NewMVec3(), "足11右"},
		{"元今右膝1", rootBone.Index(), mmath.NewMVec3(), "足11右"},
		{"先今右膝1", rootBone.Index(), mmath.NewMVec3(), "足11右"},
		{"元今右足首1", rootBone.Index(), mmath.NewMVec3(), "足11右"},
		{"先今右足首1", rootBone.Index(), mmath.NewMVec3(), "足11右"},
		{"元今右足EX1", rootBone.Index(), mmath.NewMVec3(), "足11右"},
		{"先今右足EX1", rootBone.Index(), mmath.NewMVec3(), "足11右"},
		{"元今右爪先D1", rootBone.Index(), mmath.NewMVec3(), "足11右"},
		{"先今右爪先D1", rootBone.Index(), mmath.NewMVec3(), "足11右"},
		{"元今右爪先PD1", rootBone.Index(), mmath.NewMVec3(), "足11右"},
		{"先今右爪先PD1", rootBone.Index(), mmath.NewMVec3(), "足11右"},
		{"元今右爪先CD1", rootBone.Index(), mmath.NewMVec3(), "足11右"},
		{"先今右爪先CD1", rootBone.Index(), mmath.NewMVec3(), "足11右"},
		{"元今右踵D1", rootBone.Index(), mmath.NewMVec3(), "足11右"},
		{"先今右踵D1", rootBone.Index(), mmath.NewMVec3(), "足11右"},
		// 上半身補正
		{"元今上半身", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"元今上半身2", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"元今首根元", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"元今首", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"元今左腕", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"元今右腕", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"先今上半身", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"先今上半身2", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"先今首根元", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"先今首", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"先今左腕", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"先今右腕", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"先理首根元", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"先結上半身", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"先結上半身2", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"先結首根元", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"先結首", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"先結左腕", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		{"先結右腕", rootBone.Index(), mmath.NewMVec3(), "上半身02"},
		// 肩補正
		{"左腕先肩", rootBone.Index(), mmath.NewMVec3(), "肩02_左"},
		{"左腕比率先肩", rootBone.Index(), mmath.NewMVec3(), "肩02_左"},
		{"左腕Y固定先肩", rootBone.Index(), mmath.NewMVec3(), "肩02_左"},
		{"左腕理想先肩", rootBone.Index(), mmath.NewMVec3(), "肩02_左"},
		{"左手首先肩", rootBone.Index(), mmath.NewMVec3(), "肩02_左"},
		{"左肩結果先肩", rootBone.Index(), mmath.NewMVec3(), "肩02_左"},
		{"左腕結果先肩", rootBone.Index(), mmath.NewMVec3(), "肩02_左"},
		{"左ひじ結果先肩", rootBone.Index(), mmath.NewMVec3(), "肩02_左"},
		{"左手首結果先肩", rootBone.Index(), mmath.NewMVec3(), "肩02_左"},
		{"右腕先肩", rootBone.Index(), mmath.NewMVec3(), "肩02_右"},
		{"右腕比率先肩", rootBone.Index(), mmath.NewMVec3(), "肩02_右"},
		{"右腕Y固定先肩", rootBone.Index(), mmath.NewMVec3(), "肩02_右"},
		{"右腕理想先肩", rootBone.Index(), mmath.NewMVec3(), "肩02_右"},
		{"右手首先肩", rootBone.Index(), mmath.NewMVec3(), "肩02_右"},
		{"右肩結果先肩", rootBone.Index(), mmath.NewMVec3(), "肩02_右"},
		{"右腕結果先肩", rootBone.Index(), mmath.NewMVec3(), "肩02_右"},
		{"右ひじ結果先肩", rootBone.Index(), mmath.NewMVec3(), "肩02_右"},
		{"右手首結果先肩", rootBone.Index(), mmath.NewMVec3(), "肩02_右"},
		// 位置合わせ
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
func (ss *SizingSet) insertShortageConfigBones(
	vertices *pmx.Vertices, bones *pmx.Bones, displaySlots *pmx.DisplaySlots,
) error {

	// 体幹系
	for _, funcs := range [][]func() (*pmx.Bone, error){
		{bones.GetRoot, bones.CreateRoot},
		{bones.GetBodyAxis, bones.CreateBodyAxis},
		{bones.GetTrunkRoot, bones.CreateTrunkRoot},
		{bones.GetLowerRoot, bones.CreateLowerRoot},
		{bones.GetLegCenter, bones.CreateLegCenter},
		{bones.GetUpperRoot, bones.CreateUpperRoot},
		{bones.GetNeckRoot, bones.CreateNeckRoot},
		{bones.GetHeadTail, bones.CreateHeadTail},
	} {
		getFunc := funcs[0]
		createFunc := funcs[1]

		if bone, err := getFunc(); err != nil && merr.IsNameNotFoundError(err) && bone == nil {
			if bone, err := createFunc(); err == nil && bone != nil {
				bone.IsSystem = true
				if err := bones.Insert(bone); err != nil {
					return err
				} else {
					switch bone.Name() {
					case pmx.ROOT.String():
						// 全ての親は親が-1のボーンを全部対象
						bones.ForEach(func(i int, b *pmx.Bone) bool {
							if b.ParentIndex == -1 && b.Index() != bone.Index() {
								b.ParentIndex = bone.Index()
							}
							return true
						})
					default:
						bones.SetParentFromConfig(bone)
					}

					// 再セットアップ
					bones.Setup()

					if !bone.IsVisible() {
						continue
					}
					config := bone.Config()
					if config == nil {
						continue
					}
					configDisplaySlot := config.DisplaySlot
					if configDisplaySlot == "" {
						continue
					}
					displaySlotBoneName := configDisplaySlot.StringFromDirection(bone.Direction())
					displaySlots.ForEach(func(index int, ds *pmx.DisplaySlot) bool {
						for _, r := range ds.References {
							if r.DisplayType != pmx.DISPLAY_TYPE_BONE {
								continue
							}
							if b, err := bones.Get(r.DisplayIndex); err == nil && b != nil {
								if b.Name() == displaySlotBoneName {
									// 想定ボーンと同じ表示枠に追加する
									ds.References = append(ds.References, pmx.NewDisplaySlotReferenceByValues(pmx.DISPLAY_TYPE_BONE, bone.Index()))
									return false
								}
							}
						}
						return true
					})
				}
			} else if merr.IsParentNotFoundError(err) {
				// 何もしない
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
			{bones.GetHip, bones.CreateHip},
			{bones.GetLegRoot, bones.CreateLegRoot},
			{bones.GetLegD, bones.CreateLegD},
			{bones.GetKneeD, bones.CreateKneeD},
			{bones.GetAnkleD, bones.CreateAnkleD},
			{bones.GetToeT, bones.CreateToeT},
			{bones.GetToeP, bones.CreateToeP},
			{bones.GetToeC, bones.CreateToeC},
			{bones.GetHeel, bones.CreateHeel},
			{bones.GetAnkleDGround, bones.CreateAnkleDGround},
			{bones.GetToeEx, bones.CreateToeEx},
			{bones.GetToeTD, bones.CreateToeTD},
			{bones.GetToePD, bones.CreateToePD},
			{bones.GetToeCD, bones.CreateToeCD},
			{bones.GetHeelD, bones.CreateHeelD},
		} {
			getFunc := funcs[0]
			createFunc := funcs[1]

			if bone, err := getFunc(direction); err != nil && merr.IsNameNotFoundError(err) && bone == nil {
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
						bones.SetParentFromConfig(bone)

						// 再セットアップ
						bones.Setup()

						if !bone.IsVisible() {
							continue
						}
						config := bone.Config()
						if config == nil {
							continue
						}
						configDisplaySlot := config.DisplaySlot
						if configDisplaySlot == "" {
							continue
						}
						displaySlotBoneName := configDisplaySlot.StringFromDirection(bone.Direction())
						displaySlots.ForEach(func(index int, ds *pmx.DisplaySlot) bool {
							for _, r := range ds.References {
								if r.DisplayType != pmx.DISPLAY_TYPE_BONE {
									continue
								}
								if b, err := bones.Get(r.DisplayIndex); err == nil && b != nil {
									if b.Name() == displaySlotBoneName {
										// 想定ボーンと同じ表示枠に追加する
										ds.References = append(ds.References, pmx.NewDisplaySlotReferenceByValues(pmx.DISPLAY_TYPE_BONE, bone.Index()))
										return false
									}
								}
							}
							return true
						})
					}
				} else if merr.IsParentNotFoundError(err) {
					// 何もしない
				} else {
					return err
				}
			} else if err != nil {
				return err
			}
		}

		{
			// 親指0
			if bone, err := bones.GetThumb(direction, 0); err != nil && merr.IsNameNotFoundError(err) && bone == nil {
				if thumb0, err := bones.CreateThumb0(direction); err == nil && thumb0 != nil {
					if err := bones.Insert(thumb0); err != nil {
						return err
					}
					if thumb1, err := bones.GetThumb(direction, 1); err == nil && thumb1 != nil {
						thumb1.ParentIndex = thumb0.Index()
					}
					bones.Setup()
				} else if merr.IsParentNotFoundError(err) {
					// 何もしない
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
