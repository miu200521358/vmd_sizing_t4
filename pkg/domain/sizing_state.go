package domain

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"sync"

	"github.com/miu200521358/mlib_go/pkg/config/mi18n"
	"github.com/miu200521358/mlib_go/pkg/config/mlog"
	"github.com/miu200521358/mlib_go/pkg/domain/pmx"
	"github.com/miu200521358/mlib_go/pkg/domain/vmd"
	"github.com/miu200521358/mlib_go/pkg/infrastructure/repository"
	"github.com/miu200521358/mlib_go/pkg/interface/controller"
	"github.com/miu200521358/mlib_go/pkg/interface/controller/widget"
	"github.com/miu200521358/walk/pkg/walk"
)

type SizingState struct {
	AddSetButton            *widget.MPushButton  // セット追加ボタン
	ResetSetButton          *widget.MPushButton  // セットリセットボタン
	SaveSetButton           *widget.MPushButton  // セット保存ボタン
	LoadSetButton           *widget.MPushButton  // セット読込ボタン
	NavToolBar              *walk.ToolBar        // セットツールバー
	currentIndex            int                  // 現在のインデックス
	OriginalMotionPicker    *widget.FilePicker   // 元モーション
	OriginalModelPicker     *widget.FilePicker   // 元モデル
	SizingModelPicker       *widget.FilePicker   // サイジング先モデル
	SizingMotionPicker      *widget.FilePicker   // 出力モーション
	OutputModelPicker       *widget.FilePicker   // 出力モデル
	AdoptSizingCheck        *walk.CheckBox       // サイジング反映チェック
	AdoptAllCheck           *walk.CheckBox       // 全セット反映チェック
	TerminateButton         *widget.MPushButton  // 終了ボタン
	SizingLegCheck          *walk.CheckBox       // 足チェック
	SizingUpperCheck        *walk.CheckBox       // 上半身チェック
	SizingShoulderCheck     *walk.CheckBox       // 肩チェック
	SizingArmStanceCheck    *walk.CheckBox       // 腕チェック
	SizingFingerStanceCheck *walk.CheckBox       // 指チェック
	SizingArmTwistCheck     *walk.CheckBox       // 腕捩りチェック
	SizingReductionCheck    *walk.CheckBox       // 間引きチェック
	Player                  *widget.MotionPlayer // モーションプレイヤー
	SizingSets              []*SizingSet         `json:"sizing_sets"` // サイジングセット
}

func (ss *SizingState) AddAction() {
	index := ss.NavToolBar.Actions().Len()

	action := ss.newAction(index)
	ss.NavToolBar.Actions().Add(action)
	ss.ChangeCurrentAction(index)
}

func (ss *SizingState) newAction(index int) *walk.Action {
	action := walk.NewAction()
	action.SetCheckable(true)
	action.SetExclusive(true)
	action.SetText(fmt.Sprintf(" No. %d ", index+1))

	action.Triggered().Attach(func() {
		ss.ChangeCurrentAction(index)
	})

	return action
}

func (ss *SizingState) ResetSet() {
	// 一旦全部削除
	for range ss.NavToolBar.Actions().Len() {
		index := ss.NavToolBar.Actions().Len() - 1
		ss.SizingSets[index].Delete()
		ss.NavToolBar.Actions().RemoveAt(index)
	}

	ss.SizingSets = make([]*SizingSet, 0)
	ss.currentIndex = -1

	// 1セット追加
	ss.SizingSets = append(ss.SizingSets, NewSizingSet(len(ss.SizingSets)))
	ss.AddAction()
}

func (ss *SizingState) ChangeCurrentAction(index int) {
	// 一旦すべてのチェックを外す
	for i := range ss.NavToolBar.Actions().Len() {
		ss.NavToolBar.Actions().At(i).SetChecked(false)
	}

	// 該当INDEXのみチェックON
	ss.currentIndex = index
	ss.NavToolBar.Actions().At(index).SetChecked(true)

	// サイジングセットの情報を表示
	ss.OriginalMotionPicker.ChangePath(ss.CurrentSet().OriginalMotionPath)
	ss.OriginalModelPicker.ChangePath(ss.CurrentSet().OriginalModelPath)
	ss.SizingModelPicker.ChangePath(ss.CurrentSet().SizingModelPath)
	ss.SizingMotionPicker.ChangePath(ss.CurrentSet().SizingMotionPath)
	ss.OutputModelPicker.ChangePath(ss.CurrentSet().OutputModelPath)

	// サイジングオプションの情報を表示
	ss.SizingLegCheck.SetChecked(ss.CurrentSet().IsSizingLeg)
	ss.SizingUpperCheck.SetChecked(ss.CurrentSet().IsSizingUpper)
	ss.SizingShoulderCheck.SetChecked(ss.CurrentSet().IsSizingShoulder)
	ss.SizingArmStanceCheck.SetChecked(ss.CurrentSet().IsSizingArmStance)
	ss.SizingFingerStanceCheck.SetChecked(ss.CurrentSet().IsSizingFingerStance)
	ss.SizingArmTwistCheck.SetChecked(ss.CurrentSet().IsSizingArmTwist)
	// ss.SizingReductionCheck.SetChecked(ss.CurrentSet().IsSizingReduction)
}

func (ss *SizingState) SetCurrentIndex(index int) {
	ss.currentIndex = index
}

func (ss *SizingState) CurrentIndex() int {
	return ss.currentIndex
}

func (ss *SizingState) CurrentSet() *SizingSet {
	return ss.SizingSets[ss.currentIndex]
}

// SaveSet セット情報を保存
func (ss *SizingState) SaveSet(jsonPath string) {
	if strings.ToLower(filepath.Ext(jsonPath)) != ".json" {
		// 拡張子が.jsonでない場合は付与
		jsonPath += ".json"
	}

	// セット情報をJSONに変換してファイルダイアログで選択した箇所に保存
	if output, err := json.Marshal(ss.SizingSets); err == nil && len(output) > 0 {
		if err := os.WriteFile(jsonPath, output, 0644); err == nil {
			mlog.I(mi18n.T("サイジングセット保存成功"), map[string]any{"Path": jsonPath})
		} else {
			mlog.E(mi18n.T("サイジングセット保存失敗エラー"), map[string]any{"Error": err.Error()})
		}
	} else {
		mlog.E(mi18n.T("サイジングセット保存失敗エラー"), map[string]any{"Error": err.Error()})
	}
}

// LoadSet セット情報を読み込む
func (ss *SizingState) LoadSet(jsonPath string) {
	// セット情報をJSONから読み込んでセット情報を更新
	if input, err := os.ReadFile(jsonPath); err == nil && len(input) > 0 {
		if err := json.Unmarshal(input, &ss.SizingSets); err == nil {
			mlog.I(mi18n.T("サイジングセット読込成功"), map[string]any{"Path": jsonPath})
		} else {
			mlog.E(mi18n.T("サイジングセット読込失敗エラー"), map[string]any{"Error": err.Error()})
		}
	} else {
		mlog.E(mi18n.T("サイジングセット読込失敗エラー"), map[string]any{"Error": err.Error()})
	}
}

// LoadOriginalModel 元モデルを読み込む
func (sizingState *SizingState) LoadSizingModel(
	cw *controller.ControlWindow, rep repository.IRepository, path string,
) {
	if path == "" {
		cw.StoreModel(0, sizingState.CurrentIndex(), nil)
		sizingState.CurrentSet().SetSizingModel(nil)
		return
	}

	if data, err := rep.Load(path); err == nil {
		model := data.(*pmx.PmxModel)
		if err := model.Bones.InsertShortageBones(); err != nil {
			mlog.ET(mi18n.T("システム用ボーン追加失敗"), err.Error())

			cw.StoreModel(0, sizingState.CurrentIndex(), nil)
			sizingState.CurrentSet().SetSizingModel(nil)
		} else {
			cw.StoreModel(0, sizingState.CurrentIndex(), model)
			sizingState.CurrentSet().SetSizingModel(model)

			outputPath := sizingState.CurrentSet().CreateOutputModelPath()
			sizingState.CurrentSet().OutputModelPath = outputPath
			sizingState.OutputModelPicker.ChangePath(outputPath)
		}
	} else {
		mlog.ET(mi18n.T("読み込み失敗"), err.Error())

		cw.StoreModel(0, sizingState.CurrentIndex(), nil)
		sizingState.CurrentSet().SetSizingModel(nil)
	}
}

// LoadOriginalModel オリジナルモデルを読み込む
func (sizingState *SizingState) LoadOriginalModel(
	cw *controller.ControlWindow, rep repository.IRepository, path string,
) {
	if path == "" {
		cw.StoreModel(1, sizingState.CurrentIndex(), nil)
		sizingState.CurrentSet().SetOriginalModel(nil)
		return
	}

	if data, err := rep.Load(path); err == nil {
		model := data.(*pmx.PmxModel)
		if err := model.Bones.InsertShortageBones(); err != nil {
			mlog.ET(mi18n.T("システム用ボーン追加失敗"), err.Error())

			cw.StoreModel(1, sizingState.CurrentIndex(), nil)
			sizingState.CurrentSet().SetOriginalModel(nil)
		} else {
			cw.StoreModel(1, sizingState.CurrentIndex(), model)
			sizingState.CurrentSet().SetOriginalModel(model)
		}
	} else {
		mlog.ET(mi18n.T("読み込み失敗"), err.Error())

		cw.StoreModel(1, sizingState.CurrentIndex(), nil)
		sizingState.CurrentSet().SetOriginalModel(nil)
	}
}

// LoadSizingMotion サイジングモーションを読み込む
func (sizingState *SizingState) LoadSizingMotion(
	cw *controller.ControlWindow, rep repository.IRepository, path string,
) {
	if path == "" {
		cw.StoreMotion(1, sizingState.CurrentIndex(), nil)
		sizingState.CurrentSet().SetOriginalMotion(nil)

		cw.StoreMotion(0, sizingState.CurrentIndex(), nil)
		sizingState.CurrentSet().SetSizingMotion(nil)
		return
	}

	var wg sync.WaitGroup
	var originalMotion, sizingMotion *vmd.VmdMotion

	wg.Add(1)
	go func() {
		defer wg.Done()

		vmdRep := repository.NewVmdVpdRepository()
		if data, err := vmdRep.Load(path); err == nil {
			originalMotion = data.(*vmd.VmdMotion)
		} else {
			mlog.ET(mi18n.T("読み込み失敗"), err.Error())
		}
	}()

	wg.Add(1)
	go func() {
		defer wg.Done()

		vmdRep := repository.NewVmdVpdRepository()
		if data, err := vmdRep.Load(path); err == nil {
			sizingMotion = data.(*vmd.VmdMotion)
		} else {
			mlog.ET(mi18n.T("読み込み失敗"), err.Error())
		}
	}()

	wg.Wait()

	if originalMotion != nil {
		sizingState.Player.Reset(originalMotion.MaxFrame())
	}

	cw.StoreMotion(1, sizingState.CurrentIndex(), originalMotion)
	sizingState.CurrentSet().SetOriginalMotion(originalMotion)

	cw.StoreMotion(0, sizingState.CurrentIndex(), sizingMotion)
	sizingState.CurrentSet().SetSizingMotion(sizingMotion)

	outputPath := sizingState.CurrentSet().CreateOutputMotionPath()
	sizingState.CurrentSet().SizingMotionPath = outputPath
	sizingState.SizingMotionPicker.ChangePath(outputPath)
}

// SetSizingEnabled サイジング有効無効設定
func (sizingState *SizingState) SetSizingEnabled(enabled bool) {
	sizingState.AddSetButton.SetEnabled(enabled)
	sizingState.ResetSetButton.SetEnabled(enabled)
	sizingState.SaveSetButton.SetEnabled(enabled)
	sizingState.LoadSetButton.SetEnabled(enabled)

	sizingState.OriginalMotionPicker.SetEnabled(enabled)
	sizingState.OriginalModelPicker.SetEnabled(enabled)
	sizingState.SizingModelPicker.SetEnabled(enabled)
	sizingState.SizingMotionPicker.SetEnabled(enabled)
	sizingState.OutputModelPicker.SetEnabled(enabled)

	sizingState.SizingLegCheck.SetEnabled(enabled)
	sizingState.SizingUpperCheck.SetEnabled(enabled)
	sizingState.SizingShoulderCheck.SetEnabled(enabled)
	sizingState.SizingArmStanceCheck.SetEnabled(enabled)
	sizingState.SizingFingerStanceCheck.SetEnabled(enabled)
	sizingState.SizingArmTwistCheck.SetEnabled(enabled)
	sizingState.SizingReductionCheck.SetEnabled(enabled)
}
