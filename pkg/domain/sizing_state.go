package domain

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"strings"

	"github.com/miu200521358/mlib_go/pkg/config/mi18n"
	"github.com/miu200521358/mlib_go/pkg/config/mlog"
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
	OutputMotionPicker      *widget.FilePicker   // 出力モーション
	OutputModelPicker       *widget.FilePicker   // 出力モデル
	AdoptSizingCheck        *walk.CheckBox       // サイジング反映チェック
	AdoptAllCheck           *walk.CheckBox       // 全セット反映チェック
	TerminateButton         *widget.MPushButton  // 終了ボタン
	SaveButton              *widget.MPushButton  // 保存ボタン
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
	ss.OutputMotionPicker.ChangePath(ss.CurrentSet().OutputMotionPath)
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
			mlog.I(mi18n.T("サイジングセット保存成功", map[string]any{"Path": jsonPath}))
		} else {
			mlog.E(mi18n.T("サイジングセット保存失敗エラー", map[string]any{"Error": err.Error()}))
		}
	} else {
		mlog.E(mi18n.T("サイジングセット保存失敗エラー", map[string]any{"Error": err.Error()}))
	}
}

// LoadSet セット情報を読み込む
func (ss *SizingState) LoadSet(jsonPath string) {
	// セット情報をJSONから読み込んでセット情報を更新
	if input, err := os.ReadFile(jsonPath); err == nil && len(input) > 0 {
		if err := json.Unmarshal(input, &ss.SizingSets); err == nil {
			mlog.I(mi18n.T("サイジングセット読込成功", map[string]any{"Path": jsonPath}))
		} else {
			mlog.E(mi18n.T("サイジングセット読込失敗エラー", map[string]any{"Error": err.Error()}))
		}
	} else {
		mlog.E(mi18n.T("サイジングセット読込失敗エラー", map[string]any{"Error": err.Error()}))
	}
}

// LoadOriginalModel 元モデルを読み込む
func (sizingState *SizingState) LoadOriginalModel(
	cw *controller.ControlWindow, path string,
) {
	sizingState.CurrentSet().LoadOriginalModel(path)

	cw.StoreModel(1, sizingState.CurrentIndex(), sizingState.CurrentSet().OriginalModel)

	cw.StoreMotion(0, sizingState.CurrentIndex(), sizingState.CurrentSet().OutputMotion)
	cw.StoreMotion(1, sizingState.CurrentIndex(), sizingState.CurrentSet().OriginalMotion)
}

// LoadSizingModel サイジング先モデルを読み込む
func (sizingState *SizingState) LoadSizingModel(
	cw *controller.ControlWindow, path string,
) {
	sizingState.CurrentSet().LoadSizingModel(path)

	cw.StoreModel(0, sizingState.CurrentIndex(), sizingState.CurrentSet().SizingModel)

	cw.StoreMotion(0, sizingState.CurrentIndex(), sizingState.CurrentSet().OutputMotion)
	cw.StoreMotion(1, sizingState.CurrentIndex(), sizingState.CurrentSet().OriginalMotion)

	sizingState.OutputModelPicker.SetPath(sizingState.CurrentSet().OutputModelPath)
}

// LoadSizingMotion サイジングモーションを読み込む
func (sizingState *SizingState) LoadSizingMotion(
	cw *controller.ControlWindow, path string,
) {
	sizingState.CurrentSet().LoadMotion(path)

	cw.StoreMotion(0, sizingState.CurrentIndex(), sizingState.CurrentSet().OutputMotion)
	cw.StoreMotion(1, sizingState.CurrentIndex(), sizingState.CurrentSet().OriginalMotion)

	if sizingState.CurrentSet().OriginalMotion != nil {
		sizingState.Player.Reset(sizingState.CurrentSet().OriginalMotion.MaxFrame())
	}

	sizingState.OutputMotionPicker.ChangePath(sizingState.CurrentSet().OutputMotionPath)
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
	sizingState.OutputMotionPicker.SetEnabled(enabled)
	sizingState.OutputModelPicker.SetEnabled(enabled)

	sizingState.Player.SetEnabled(enabled)

	sizingState.SetSizingOptionEnabled(enabled)
}

func (sizingState *SizingState) SetSizingOptionEnabled(enabled bool) {
	sizingState.AdoptSizingCheck.SetEnabled(enabled)
	sizingState.AdoptAllCheck.SetEnabled(enabled)
	sizingState.TerminateButton.SetEnabled(enabled)
	sizingState.SaveButton.SetEnabled(enabled)

	sizingState.SizingLegCheck.SetEnabled(enabled)
	sizingState.SizingUpperCheck.SetEnabled(enabled)
	sizingState.SizingShoulderCheck.SetEnabled(enabled)
	sizingState.SizingArmStanceCheck.SetEnabled(enabled)
	sizingState.SizingFingerStanceCheck.SetEnabled(enabled)
	sizingState.SizingArmTwistCheck.SetEnabled(enabled)
}
