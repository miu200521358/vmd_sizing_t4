package ui

import (
	"miu200521358/vmd_sizing_t4.git/pkg/domain"

	"github.com/miu200521358/mlib_go/pkg/config/mi18n"
	"github.com/miu200521358/mlib_go/pkg/config/mlog"
	"github.com/miu200521358/mlib_go/pkg/domain/pmx"
	"github.com/miu200521358/mlib_go/pkg/domain/vmd"
	"github.com/miu200521358/mlib_go/pkg/infrastructure/repository"
	"github.com/miu200521358/mlib_go/pkg/interface/controller"
	"github.com/miu200521358/mlib_go/pkg/interface/controller/widget"
	"github.com/miu200521358/walk/pkg/declarative"
	"github.com/miu200521358/walk/pkg/walk"
)

func NewSizingPage(mWidgets *controller.MWidgets) declarative.TabPage {
	var fileTab *walk.TabPage
	sizingState := new(domain.SizingState)

	player := widget.NewMotionPlayer()

	originalVmdPicker := widget.NewVmdVpdLoadFilePicker(
		"vmd",
		mi18n.T("サイジング対象モーション(Vmd/Vpd)"),
		mi18n.T("サイジング対象モーションツールチップ"),
		func(cw *controller.ControlWindow, rep repository.IRepository, path string) {
			if path == "" {
				cw.StoreMotion(0, sizingState.CurrentIndex, nil)
				cw.StoreMotion(1, sizingState.CurrentIndex, nil)
				return
			}

			if data, err := rep.Load(path); err == nil {
				motion := data.(*vmd.VmdMotion)
				player.Reset(motion.MaxFrame())
				cw.StoreMotion(0, sizingState.CurrentIndex, motion)
			} else {
				mlog.ET(mi18n.T("読み込み失敗"), err.Error())
			}

			if data, err := rep.Load(path); err == nil {
				motion := data.(*vmd.VmdMotion)
				cw.StoreMotion(1, sizingState.CurrentIndex, motion)
			} else {
				mlog.ET(mi18n.T("読み込み失敗"), err.Error())
			}
		},
	)

	originalPmxPicker := widget.NewPmxJsonLoadFilePicker(
		"org_pmx",
		mi18n.T("モーション作成元モデル(Json/Pmx)"),
		mi18n.T("モーション作成元モデルツールチップ"),
		func(cw *controller.ControlWindow, rep repository.IRepository, path string) {
			if data, err := rep.Load(path); err == nil {
				model := data.(*pmx.PmxModel)
				if err := model.Bones.InsertShortageBones(); err != nil {
					mlog.ET(mi18n.T("システム用ボーン追加失敗"), err.Error())
				} else {
					cw.StoreModel(0, sizingState.CurrentIndex, model)
				}
			} else {
				mlog.ET(mi18n.T("読み込み失敗"), err.Error())
				cw.StoreModel(0, 0, nil)
			}
		},
	)

	sizingPmxPicker := widget.NewPmxJsonLoadFilePicker(
		"rep_pmx",
		mi18n.T("サイジング先モデル(Pmx)"),
		mi18n.T("サイジング先モデルツールチップ"),
		func(cw *controller.ControlWindow, rep repository.IRepository, path string) {
			if data, err := rep.Load(path); err == nil {
				model := data.(*pmx.PmxModel)
				if err := model.Bones.InsertShortageBones(); err != nil {
					mlog.ET(mi18n.T("システム用ボーン追加失敗"), err.Error())
				} else {
					cw.StoreModel(0, sizingState.CurrentIndex, model)
				}
			} else {
				mlog.ET(mi18n.T("読み込み失敗"), err.Error())
				cw.StoreModel(0, 0, nil)
			}
		},
	)

	outputVmdPicker := widget.NewVmdSaveFilePicker(
		mi18n.T("出力モーション(Vmd)"),
		mi18n.T("出力モーションツールチップ"),
		func(cw *controller.ControlWindow, rep repository.IRepository, path string) {
			motion := cw.LoadMotion(0, sizingState.CurrentIndex)
			if motion == nil {
				return
			}

			if err := rep.Save(path, motion, false); err != nil {
				mlog.ET(mi18n.T("保存失敗"), err.Error())
			}
		},
	)

	outputPmxPicker := widget.NewPmxSaveFilePicker(
		mi18n.T("出力モデル(Pmx)"),
		mi18n.T("出力モデルツールチップ"),
		func(cw *controller.ControlWindow, rep repository.IRepository, path string) {
			model := cw.LoadModel(0, sizingState.CurrentIndex)
			if model == nil {
				return
			}

			if err := rep.Save(path, model, false); err != nil {
				mlog.ET(mi18n.T("保存失敗"), err.Error())
			}
		},
	)

	mWidgets.Widgets = append(mWidgets.Widgets, player, originalVmdPicker, originalPmxPicker, sizingPmxPicker, outputVmdPicker, outputPmxPicker)
	mWidgets.SetOnLoaded(func() {
		sizingState.AddSet()
	})

	return declarative.TabPage{
		Title:    mi18n.T("ファイル"),
		AssignTo: &fileTab,
		Layout:   declarative.VBox{},
		Background: declarative.SolidColorBrush{
			Color: controller.ColorTabBackground,
		},
		Children: []declarative.Widget{
			declarative.Composite{
				Layout:  declarative.HBox{},
				MinSize: declarative.Size{Width: 200, Height: 40},
				MaxSize: declarative.Size{Width: 5120, Height: 40},
				Children: []declarative.Widget{
					declarative.HSpacer{},
					// サイジングセット追加ボタン
					declarative.PushButton{
						Text: mi18n.T("サイジングセット追加"),
						OnClicked: func() {
							sizingState.AddSet()
						},
						MinSize: declarative.Size{Width: 130, Height: 20},
						MaxSize: declarative.Size{Width: 130, Height: 20},
					},
					// サイジングセット全削除ボタン
					declarative.PushButton{
						Text: mi18n.T("サイジングセット全削除"),
						OnClicked: func() {
							// toolState.resetSizingSet()
						},
						MinSize: declarative.Size{Width: 130, Height: 20},
						MaxSize: declarative.Size{Width: 130, Height: 20},
					},
					// サイジングセット設定読み込みボタン
					declarative.PushButton{
						Text: mi18n.T("サイジングセット設定読込"),
						OnClicked: func() {
							// toolState.loadSizingSet()
						},
						MinSize: declarative.Size{Width: 130, Height: 20},
						MaxSize: declarative.Size{Width: 130, Height: 20},
					},
				},
			},
			// サイジングセットスクロール
			declarative.ScrollView{
				Layout:        declarative.VBox{},
				MinSize:       declarative.Size{Width: 200, Height: 40},
				MaxSize:       declarative.Size{Width: 5120, Height: 40},
				VerticalFixed: true,
				Children: []declarative.Widget{
					// ナビゲーション用ツールバー
					declarative.ToolBar{
						AssignTo:           &sizingState.NavToolBar,
						MinSize:            declarative.Size{Width: 200, Height: 25},
						MaxSize:            declarative.Size{Width: 5120, Height: 25},
						DefaultButtonWidth: 200,
						Background: declarative.SolidColorBrush{
							Color: controller.ColorNavBackground,
						},
						Orientation: walk.Horizontal,
						ButtonStyle: declarative.ToolBarButtonTextOnly,
					},
				},
			},
			// サイジングセットごとのサイジング内容
			declarative.ScrollView{
				Layout:  declarative.VBox{},
				MinSize: declarative.Size{Width: 126, Height: 512},
				MaxSize: declarative.Size{Width: 2560, Height: 5120},
				Children: []declarative.Widget{
					originalVmdPicker.Widgets(),
					originalPmxPicker.Widgets(),
					sizingPmxPicker.Widgets(),
					declarative.VSeparator{},
					outputVmdPicker.Widgets(),
					outputPmxPicker.Widgets(),
					declarative.VSeparator{},
				},
			},
			player.Widgets(),
		},
	}
}
