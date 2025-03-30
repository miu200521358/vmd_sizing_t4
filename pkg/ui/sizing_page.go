package ui

import (
	"miu200521358/vmd_sizing_t4/pkg/domain"
	"miu200521358/vmd_sizing_t4/pkg/usecase"
	"path/filepath"

	"github.com/miu200521358/mlib_go/pkg/config/mconfig"
	"github.com/miu200521358/mlib_go/pkg/config/mi18n"
	"github.com/miu200521358/mlib_go/pkg/config/mlog"
	"github.com/miu200521358/mlib_go/pkg/domain/pmx"
	"github.com/miu200521358/mlib_go/pkg/infrastructure/repository"
	"github.com/miu200521358/mlib_go/pkg/interface/controller"
	"github.com/miu200521358/mlib_go/pkg/interface/controller/widget"
	"github.com/miu200521358/walk/pkg/declarative"
	"github.com/miu200521358/walk/pkg/walk"
)

func NewSizingPage(mWidgets *controller.MWidgets) declarative.TabPage {
	var sizingTab *walk.TabPage
	sizingState := new(domain.SizingState)

	sizingState.Player = widget.NewMotionPlayer()

	sizingState.OutputMotionPicker = widget.NewVmdSaveFilePicker(
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
			sizingState.LoadSizingMotion(cw, path)
		},
	)

	sizingState.OriginalModelPicker = widget.NewPmxPmxJsonLoadFilePicker(
		"org_pmx",
		mi18n.T("モーション作成元モデル(Pmx/Json)"),
		mi18n.T("モーション作成元モデルツールチップ"),
		func(cw *controller.ControlWindow, rep repository.IRepository, path string) {
			sizingState.LoadOriginalModel(cw, path)
		},
	)

	sizingState.SizingModelPicker = widget.NewPmxLoadFilePicker(
		"rep_pmx",
		mi18n.T("サイジング先モデル(Pmx)"),
		mi18n.T("サイジング先モデルツールチップ"),
		func(cw *controller.ControlWindow, rep repository.IRepository, path string) {
			sizingState.LoadSizingModel(cw, path)
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
				map[string]any{"Title": "Json"}),
			Filter:         "Json files (*.json)|*.json",
			FilterIndex:    1,
			InitialDirPath: initialDirPath,
		}
		if ok, err := dlg.ShowOpen(nil); err != nil {
			walk.MsgBox(nil, mi18n.T("ファイル選択ダイアログ選択エラー"), err.Error(), walk.MsgBoxIconError)
		} else if ok {
			sizingState.SetSizingEnabled(false)
			mconfig.SaveUserConfig("sizing_set_path", dlg.FilePath, 1)

			for n := range 2 {
				for m := range sizingState.NavToolBar.Actions().Len() {
					mWidgets.Window().StoreModel(n, m, nil)
					mWidgets.Window().StoreMotion(n, m, nil)
				}
			}

			sizingState.ResetSet()
			sizingState.LoadSet(dlg.FilePath)

			for range len(sizingState.SizingSets) - 1 {
				sizingState.AddAction()
			}

			for index := range sizingState.SizingSets {
				sizingState.ChangeCurrentAction(index)
				sizingState.OriginalMotionPicker.SetPath(sizingState.SizingSets[index].OriginalMotionPath)
				sizingState.OriginalModelPicker.SetPath(sizingState.SizingSets[index].OriginalModelPath)
				sizingState.SizingModelPicker.SetPath(sizingState.SizingSets[index].SizingModelPath)
			}

			sizingState.SetCurrentIndex(0)
			sizingState.SetSizingEnabled(true)
		}
	})

	sizingState.SaveSetButton = widget.NewMPushButton()
	sizingState.SaveSetButton.SetLabel(mi18n.T("セット設定保存"))
	sizingState.SaveSetButton.SetTooltip(mi18n.T("セット設定保存説明"))
	sizingState.SaveSetButton.SetMaxSize(declarative.Size{Width: 100, Height: 20})
	sizingState.SaveSetButton.SetOnClicked(func(cw *controller.ControlWindow) {
		// サイジング元モーションパスを初期パスとする
		initialDirPath := filepath.Dir(sizingState.CurrentSet().OriginalMotionPath)

		// ファイル選択ダイアログを開く
		dlg := walk.FileDialog{
			Title: mi18n.T(
				"ファイル選択ダイアログタイトル",
				map[string]any{"Title": "Json"}),
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

	sizingState.TerminateButton = widget.NewMPushButton()
	sizingState.TerminateButton.SetLabel(mi18n.T("処理停止"))
	sizingState.TerminateButton.SetTooltip(mi18n.T("処理停止説明"))
	sizingState.TerminateButton.SetMaxSize(declarative.Size{Width: 100, Height: 20})
	sizingState.TerminateButton.SetOnClicked(func(cw *controller.ControlWindow) {
		// 押したら非活性
		sizingState.TerminateButton.SetEnabled(false)
		for _, sizingSet := range sizingState.SizingSets {
			sizingSet.IsTerminate = true
		}
	})

	sizingState.SaveButton = widget.NewMPushButton()
	sizingState.SaveButton.SetLabel(mi18n.T("モーション保存"))
	sizingState.SaveButton.SetTooltip(mi18n.T("モーション保存説明"))
	sizingState.SaveButton.SetMinSize(declarative.Size{Width: 256, Height: 20})
	sizingState.SaveButton.SetStretchFactor(20)
	sizingState.SaveButton.SetOnClicked(func(cw *controller.ControlWindow) {
		sizingState.SetSizingEnabled(false)

		for _, sizingSet := range sizingState.SizingSets {
			if sizingSet.OutputMotionPath != "" && sizingSet.OutputMotion != nil {
				rep := repository.NewVmdRepository(true)
				if err := rep.Save(sizingSet.OutputMotionPath, sizingSet.OutputMotion, false); err != nil {
					mlog.ET(mi18n.T("保存失敗"), err.Error())
				}
			}
		}

		sizingState.SetSizingEnabled(true)
		controller.Beep()
	})

	mWidgets.Widgets = append(mWidgets.Widgets, sizingState.Player, sizingState.OriginalMotionPicker,
		sizingState.OriginalModelPicker, sizingState.SizingModelPicker, sizingState.OutputMotionPicker,
		sizingState.OutputModelPicker, sizingState.AddSetButton, sizingState.ResetSetButton,
		sizingState.LoadSetButton, sizingState.SaveSetButton, sizingState.TerminateButton, sizingState.SaveButton)
	mWidgets.SetOnLoaded(func() {
		sizingState.SizingSets = append(sizingState.SizingSets, domain.NewSizingSet(len(sizingState.SizingSets)))
		sizingState.AddAction()
		sizingState.TerminateButton.SetEnabled(false)
	})
	mWidgets.SetOnChangePlaying(func(playing bool) {
		sizingState.SetSizingOptionEnabled(!playing)
	})

	return declarative.TabPage{
		Title:    mi18n.T("サイジング"),
		AssignTo: &sizingTab,
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
						Orientation:        walk.Horizontal,
						ButtonStyle:        declarative.ToolBarButtonTextOnly,
					},
				},
			},
			// セットごとのサイジング内容
			declarative.ScrollView{
				Layout:  declarative.VBox{},
				MinSize: declarative.Size{Width: 126, Height: 450},
				MaxSize: declarative.Size{Width: 2560, Height: 5120},
				Children: []declarative.Widget{
					sizingState.OriginalMotionPicker.Widgets(),
					sizingState.OriginalModelPicker.Widgets(),
					sizingState.SizingModelPicker.Widgets(),
					declarative.VSeparator{},
					sizingState.OutputMotionPicker.Widgets(),
					sizingState.OutputModelPicker.Widgets(),
					declarative.VSeparator{},
					declarative.TextLabel{
						Text: mi18n.T("サイジングオプション"),
						OnMouseDown: func(x, y int, button walk.MouseButton) {
							mlog.ILT(mi18n.T("サイジングオプション"), mi18n.T("サイジングオプション説明"))
						},
					},
					declarative.Composite{
						Layout: declarative.Grid{Columns: 3},
						Children: []declarative.Widget{
							declarative.CheckBox{
								AssignTo:    &sizingState.AdoptSizingCheck,
								Text:        mi18n.T("即時反映"),
								ToolTipText: mi18n.T("即時反映説明"),
								Checked:     true,
								OnCheckStateChanged: func() {
									// go usecase.ExecSizing(mWidgets.Window(), sizingState)
								},
							},
							declarative.CheckBox{
								AssignTo:    &sizingState.AdoptAllCheck,
								Text:        mi18n.T("全セット反映"),
								ToolTipText: mi18n.T("全セット反映説明"),
								Checked:     true,
							},
							sizingState.TerminateButton.Widgets(),
							declarative.CheckBox{
								AssignTo:    &sizingState.SizingLegCheck,
								Text:        mi18n.T("足補正"),
								ToolTipText: mi18n.T("足補正説明"),
								OnCheckStateChanged: func() {
									changeSizingCheck(mWidgets.Window(), sizingState, pmx.LEG)
								},
							},
							declarative.CheckBox{
								AssignTo:    &sizingState.SizingUpperCheck,
								Text:        mi18n.T("上半身補正"),
								ToolTipText: mi18n.T("上半身補正説明"),
								OnCheckStateChanged: func() {
									changeSizingCheck(mWidgets.Window(), sizingState, pmx.UPPER)
								},
							},
							declarative.CheckBox{
								AssignTo:    &sizingState.SizingShoulderCheck,
								Text:        mi18n.T("肩補正"),
								ToolTipText: mi18n.T("肩補正説明"),
								OnCheckStateChanged: func() {
									changeSizingCheck(mWidgets.Window(), sizingState, pmx.SHOULDER)
								},
							},
							declarative.CheckBox{
								AssignTo:    &sizingState.SizingArmStanceCheck,
								Text:        mi18n.T("腕スタンス補正"),
								ToolTipText: mi18n.T("腕スタンス補正説明"),
								OnCheckStateChanged: func() {
									changeSizingCheck(mWidgets.Window(), sizingState, pmx.LEG)
								},
							},
							declarative.CheckBox{
								AssignTo:    &sizingState.SizingFingerStanceCheck,
								Text:        mi18n.T("指スタンス補正"),
								ToolTipText: mi18n.T("指スタンス補正説明"),
								OnCheckStateChanged: func() {
									changeSizingCheck(mWidgets.Window(), sizingState, pmx.INDEX1)
								},
							},
							declarative.CheckBox{
								AssignTo:    &sizingState.SizingArmTwistCheck,
								Text:        mi18n.T("捩り補正"),
								ToolTipText: mi18n.T("捩り補正説明"),
								OnCheckStateChanged: func() {
									changeSizingCheck(mWidgets.Window(), sizingState, pmx.ARM_TWIST)
								},
							},
						},
					},
				},
			},
			sizingState.SaveButton.Widgets(),
			sizingState.Player.Widgets(),
		},
	}
}

func changeSizingCheck(cw *controller.ControlWindow, sizingState *domain.SizingState, bone pmx.StandardBoneName) {
	startIndex := 0
	endIndex := len(sizingState.SizingSets)
	if !sizingState.AdoptAllCheck.Checked() {
		// 現在のインデックスのみ
		startIndex = sizingState.CurrentIndex()
		endIndex = startIndex + 1
	}

	for _, sizingSet := range sizingState.SizingSets[startIndex:endIndex] {
		switch bone {
		case pmx.LEG:
			sizingSet.IsSizingLeg = sizingState.SizingLegCheck.Checked()
		case pmx.UPPER:
			sizingSet.IsSizingUpper = sizingState.SizingUpperCheck.Checked()
		case pmx.SHOULDER:
			sizingSet.IsSizingShoulder = sizingState.SizingShoulderCheck.Checked()
		case pmx.ARM:
			sizingSet.IsSizingArmStance = sizingState.SizingArmStanceCheck.Checked()
		case pmx.INDEX1:
			sizingSet.IsSizingFingerStance = sizingState.SizingFingerStanceCheck.Checked()
		case pmx.ARM_TWIST:
			sizingSet.IsSizingArmTwist = sizingState.SizingArmTwistCheck.Checked()
		}

		outputPath := sizingSet.CreateOutputMotionPath()
		if outputPath != "" {
			sizingSet.OutputMotionPath = outputPath
			if sizingSet.Index == sizingState.CurrentIndex() {
				sizingState.OutputMotionPicker.SetPath(outputPath)
			}
		}
	}

	go usecase.ExecSizing(cw, sizingState)
}
