package ui

import (
	"miu200521358/vmd_sizing_t4.git/pkg/domain"
	"path/filepath"

	"github.com/miu200521358/mlib_go/pkg/config/mconfig"
	"github.com/miu200521358/mlib_go/pkg/config/mi18n"
	"github.com/miu200521358/mlib_go/pkg/config/mlog"
	"github.com/miu200521358/mlib_go/pkg/infrastructure/repository"
	"github.com/miu200521358/mlib_go/pkg/interface/controller"
	"github.com/miu200521358/mlib_go/pkg/interface/controller/widget"
	"github.com/miu200521358/walk/pkg/declarative"
	"github.com/miu200521358/walk/pkg/walk"
)

func NewSizingPage(mWidgets *controller.MWidgets) declarative.TabPage {
	var fileTab *walk.TabPage
	sizingState := new(domain.SizingState)

	sizingState.Player = widget.NewMotionPlayer()

	sizingState.SizingMotionPicker = widget.NewVmdSaveFilePicker(
		mi18n.T("出力モーション(Vmd)"),
		mi18n.T("出力モーションツールチップ"),
		func(cw *controller.ControlWindow, rep repository.IRepository, path string) {
			motion := cw.LoadMotion(0, sizingState.CurrentIndex())
			if motion == nil {
				return
			}

			if err := rep.Save(path, motion, false); err != nil {
				mlog.ET(mi18n.T("保存失敗"), err.Error())
			}
		},
	)

	sizingState.OutputModelPicker = widget.NewPmxSaveFilePicker(
		mi18n.T("出力モデル(Pmx)"),
		mi18n.T("出力モデルツールチップ"),
		func(cw *controller.ControlWindow, rep repository.IRepository, path string) {
			model := cw.LoadModel(0, sizingState.CurrentIndex())
			if model == nil {
				return
			}

			if err := rep.Save(path, model, false); err != nil {
				mlog.ET(mi18n.T("保存失敗"), err.Error())
			}
		},
	)

	sizingState.OriginalMotionPicker = widget.NewVmdVpdLoadFilePicker(
		"vmd",
		mi18n.T("サイジング対象モーション(Vmd/Vpd)"),
		mi18n.T("サイジング対象モーションツールチップ"),
		func(cw *controller.ControlWindow, rep repository.IRepository, path string) {
			sizingState.LoadSizingMotion(cw, rep, path)
		},
	)

	sizingState.OriginalModelPicker = widget.NewPmxPmxJsonLoadFilePicker(
		"org_pmx",
		mi18n.T("モーション作成元モデル(Pmx/Json)"),
		mi18n.T("モーション作成元モデルツールチップ"),
		func(cw *controller.ControlWindow, rep repository.IRepository, path string) {
			sizingState.LoadOriginalModel(cw, rep, path)
		},
	)

	sizingState.SizingModelPicker = widget.NewPmxLoadFilePicker(
		"rep_pmx",
		mi18n.T("サイジング先モデル(Pmx)"),
		mi18n.T("サイジング先モデルツールチップ"),
		func(cw *controller.ControlWindow, rep repository.IRepository, path string) {
			sizingState.LoadSizingModel(cw, rep, path)
		},
	)

	sizingState.AddSetButton = widget.NewMPushButton()
	sizingState.AddSetButton.SetLabel(mi18n.T("セット追加"))
	sizingState.AddSetButton.SetTooltip(mi18n.T("セット追加説明"))
	sizingState.AddSetButton.SetMaxSize(declarative.Size{Width: 100, Height: 20})
	sizingState.AddSetButton.SetOnClicked(func(cw *controller.ControlWindow) {
		sizingState.SizingSets = append(sizingState.SizingSets,
			domain.NewSizingSet(len(sizingState.SizingSets)))
		sizingState.AddAction()
	})

	sizingState.ResetSetButton = widget.NewMPushButton()
	sizingState.ResetSetButton.SetLabel(mi18n.T("セット全削除"))
	sizingState.ResetSetButton.SetTooltip(mi18n.T("セット全削除説明"))
	sizingState.ResetSetButton.SetMaxSize(declarative.Size{Width: 100, Height: 20})
	sizingState.ResetSetButton.SetOnClicked(func(cw *controller.ControlWindow) {
		for n := range 2 {
			for m := range sizingState.NavToolBar.Actions().Len() {
				mWidgets.Window().StoreModel(n, m, nil)
				mWidgets.Window().StoreMotion(n, m, nil)
			}
		}

		sizingState.ResetSet()
	})

	sizingState.LoadSetButton = widget.NewMPushButton()
	sizingState.LoadSetButton.SetLabel(mi18n.T("セット設定読込"))
	sizingState.LoadSetButton.SetTooltip(mi18n.T("セット設定読込説明"))
	sizingState.LoadSetButton.SetMaxSize(declarative.Size{Width: 100, Height: 20})
	sizingState.LoadSetButton.SetOnClicked(func(cw *controller.ControlWindow) {
		choices := mconfig.LoadUserConfig("sizing_set_path")
		var initialDirPath string
		if len(choices) > 0 {
			// ファイルパスからディレクトリパスを取得
			initialDirPath = filepath.Dir(choices[0])
		}

		// ファイル選択ダイアログを開く
		dlg := walk.FileDialog{
			Title: mi18n.T(
				"ファイル選択ダイアログタイトル",
				map[string]interface{}{"Title": "Json"}),
			Filter:         "Json files (*.json)|*.json",
			FilterIndex:    1,
			InitialDirPath: initialDirPath,
		}
		if ok, err := dlg.ShowOpen(nil); err != nil {
			walk.MsgBox(nil, mi18n.T("ファイル選択ダイアログ選択エラー"), err.Error(), walk.MsgBoxIconError)
		} else if ok {
			mWidgets.Window().SetEnabled(false)

			for n := range 2 {
				for m := range sizingState.NavToolBar.Actions().Len() {
					mWidgets.Window().StoreModel(n, m, nil)
					mWidgets.Window().StoreMotion(n, m, nil)
				}
			}

			sizingState.ResetSet()
			sizingState.LoadSet(dlg.FilePath)
			cw := mWidgets.Window()

			for range len(sizingState.SizingSets) - 1 {
				sizingState.AddAction()
			}

			for index := range sizingState.SizingSets {
				sizingState.SetCurrentIndex(index)
				{
					rep := repository.NewPmxRepository()
					sizingState.LoadOriginalModel(cw, rep, sizingState.SizingSets[index].OriginalModelPath)
				}
				{
					rep := repository.NewPmxPmxJsonRepository()
					sizingState.LoadSizingModel(cw, rep, sizingState.SizingSets[index].SizingModelPath)
				}
				{
					rep := repository.NewVmdVpdRepository()
					sizingState.LoadSizingMotion(cw, rep, sizingState.SizingSets[index].OriginalMotionPath)
				}
			}

			sizingState.SetCurrentIndex(0)
			mWidgets.Window().SetEnabled(true)
		}
	})

	sizingState.SaveSetButton = widget.NewMPushButton()
	sizingState.SaveSetButton.SetLabel(mi18n.T("セット設定保存"))
	sizingState.SaveSetButton.SetTooltip(mi18n.T("セット設定保存説明"))
	sizingState.SaveSetButton.SetMaxSize(declarative.Size{Width: 100, Height: 20})
	sizingState.SaveSetButton.SetOnClicked(func(cw *controller.ControlWindow) {
		choices := mconfig.LoadUserConfig("sizing_set_path")
		var initialDirPath string
		if len(choices) > 0 {
			// ファイルパスからディレクトリパスを取得
			initialDirPath = filepath.Dir(choices[0])
		}

		// ファイル選択ダイアログを開く
		dlg := walk.FileDialog{
			Title: mi18n.T(
				"ファイル選択ダイアログタイトル",
				map[string]interface{}{"Title": "Json"}),
			Filter:         "Json files (*.json)|*.json",
			FilterIndex:    1,
			InitialDirPath: initialDirPath,
		}
		if ok, err := dlg.ShowSave(nil); err != nil {
			walk.MsgBox(nil, mi18n.T("ファイル選択ダイアログ選択エラー"), err.Error(), walk.MsgBoxIconError)
		} else if ok {
			sizingState.SaveSet(dlg.FilePath)
			mconfig.SaveUserConfig("sizing_set_path", dlg.FilePath, 1)
		}
	})

	mWidgets.Widgets = append(mWidgets.Widgets, sizingState.Player, sizingState.OriginalMotionPicker,
		sizingState.OriginalModelPicker, sizingState.SizingModelPicker, sizingState.SizingMotionPicker,
		sizingState.OutputModelPicker, sizingState.AddSetButton, sizingState.ResetSetButton,
		sizingState.LoadSetButton, sizingState.SaveSetButton)
	mWidgets.SetOnLoaded(func() {
		sizingState.SizingSets = append(sizingState.SizingSets, domain.NewSizingSet(len(sizingState.SizingSets)))
		sizingState.AddAction()
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
					sizingState.AddSetButton.Widgets(),
					sizingState.ResetSetButton.Widgets(),
					sizingState.LoadSetButton.Widgets(),
					sizingState.SaveSetButton.Widgets(),
				},
			},
			// セットスクロール
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
			// セットごとのサイジング内容
			declarative.ScrollView{
				Layout:  declarative.VBox{},
				MinSize: declarative.Size{Width: 126, Height: 512},
				MaxSize: declarative.Size{Width: 2560, Height: 5120},
				Children: []declarative.Widget{
					sizingState.OriginalMotionPicker.Widgets(),
					sizingState.OriginalModelPicker.Widgets(),
					sizingState.SizingModelPicker.Widgets(),
					declarative.VSeparator{},
					sizingState.SizingMotionPicker.Widgets(),
					sizingState.OutputModelPicker.Widgets(),
					declarative.VSeparator{},
				},
			},
			sizingState.Player.Widgets(),
		},
	}
}
