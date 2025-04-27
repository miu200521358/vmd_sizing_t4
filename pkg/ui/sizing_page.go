package ui

import (
	"miu200521358/vmd_sizing_t4/pkg/domain"
	"miu200521358/vmd_sizing_t4/pkg/usecase"
	"path/filepath"
	"strconv"

	"github.com/miu200521358/mlib_go/pkg/config/mconfig"
	"github.com/miu200521358/mlib_go/pkg/config/merr"
	"github.com/miu200521358/mlib_go/pkg/config/mi18n"
	"github.com/miu200521358/mlib_go/pkg/config/mlog"
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
				mlog.ET(mi18n.T("保存失敗"), err, "")
				if ok := merr.ShowErrorDialog(cw.AppConfig(), err); ok {
					sizingState.SetSizingEnabled(true)
				}
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
				mlog.ET(mi18n.T("保存失敗"), err, "")
				if ok := merr.ShowErrorDialog(cw.AppConfig(), err); ok {
					sizingState.SetSizingEnabled(true)
				}
			}
		},
	)

	sizingState.OriginalMotionPicker = widget.NewVmdVpdLoadFilePicker(
		"vmd",
		mi18n.T("サイジング対象モーション(Vmd/Vpd)"),
		mi18n.T("サイジング対象モーションツールチップ"),
		func(cw *controller.ControlWindow, rep repository.IRepository, path string) {
			if err := sizingState.LoadSizingMotion(cw, path, true); err != nil {
				if ok := merr.ShowErrorDialog(cw.AppConfig(), err); ok {
					sizingState.SetSizingEnabled(true)
				}
			}
		},
	)

	sizingState.OriginalModelPicker = widget.NewPmxPmxJsonLoadFilePicker(
		"org_pmx",
		mi18n.T("モーション作成元モデル(Pmx/Json)"),
		mi18n.T("モーション作成元モデルツールチップ"),
		func(cw *controller.ControlWindow, rep repository.IRepository, path string) {
			if err := sizingState.LoadOriginalModel(cw, path); err != nil {
				if ok := merr.ShowErrorDialog(cw.AppConfig(), err); ok {
					sizingState.SetSizingEnabled(true)
				}
			}
		},
	)

	sizingState.SizingModelPicker = widget.NewPmxLoadFilePicker(
		"rep_pmx",
		mi18n.T("サイジング先モデル(Pmx)"),
		mi18n.T("サイジング先モデルツールチップ"),
		func(cw *controller.ControlWindow, rep repository.IRepository, path string) {
			if err := sizingState.LoadSizingModel(cw, path); err != nil {
				if ok := merr.ShowErrorDialog(cw.AppConfig(), err); ok {
					sizingState.SetSizingEnabled(true)
				}
			}
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
				sizingState.OriginalMotionPicker.SetForcePath(sizingState.SizingSets[index].OriginalMotionPath)
				sizingState.OriginalModelPicker.SetForcePath(sizingState.SizingSets[index].OriginalModelPath)
				sizingState.SizingModelPicker.SetForcePath(sizingState.SizingSets[index].SizingModelPath)

				sizingState.SizingArmStanceCheck.SetChecked(sizingState.SizingSets[index].IsSizingArmStance)
				sizingState.SizingLegCheck.SetChecked(sizingState.SizingSets[index].IsSizingLeg)
				sizingState.SizingUpperCheck.SetChecked(sizingState.SizingSets[index].IsSizingUpper)
				sizingState.SizingShoulderCheck.SetChecked(sizingState.SizingSets[index].IsSizingShoulder)
				sizingState.SizingFingerStanceCheck.SetChecked(sizingState.SizingSets[index].IsSizingFingerStance)
				sizingState.SizingArmTwistCheck.SetChecked(sizingState.SizingSets[index].IsSizingArmTwist)
				sizingState.SizingWristCheck.SetChecked(sizingState.SizingSets[index].IsSizingWrist)
				sizingState.ShoulderWeightEdit.ChangeText(strconv.Itoa(sizingState.SizingSets[index].ShoulderWeight))
				sizingState.ShoulderWeightSlider.ChangeValue(sizingState.SizingSets[index].ShoulderWeight)
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
					mlog.ET(mi18n.T("保存失敗"), err, "")
					if ok := merr.ShowErrorDialog(cw.AppConfig(), err); ok {
						sizingState.SetSizingEnabled(true)
					}
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
		sizingState.AdoptSizingCheck.CheckStateChanged().Attach(func() {
			changeSizingCheck(mWidgets.Window(), sizingState)
		})
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
				MinSize: declarative.Size{Width: 126, Height: 350},
				MaxSize: declarative.Size{Width: 2560, Height: 5120},
				Children: []declarative.Widget{
					sizingState.OriginalMotionPicker.Widgets(),
					sizingState.OriginalModelPicker.Widgets(),
					sizingState.SizingModelPicker.Widgets(),
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
								AssignTo:    &sizingState.SizingArmStanceCheck,
								Text:        mi18n.T("腕スタンス補正"),
								ToolTipText: mi18n.T("腕スタンス補正説明"),
								OnCheckStateChanged: func() {
									changeSizingCheck(mWidgets.Window(), sizingState)
								},
							},
							declarative.CheckBox{
								AssignTo:    &sizingState.SizingLegCheck,
								Text:        mi18n.T("足補正"),
								ToolTipText: mi18n.T("足補正説明"),
								OnCheckStateChanged: func() {
									changeSizingCheck(mWidgets.Window(), sizingState)
								},
							},
							declarative.CheckBox{
								AssignTo:    &sizingState.SizingUpperCheck,
								Text:        mi18n.T("上半身補正"),
								ToolTipText: mi18n.T("上半身補正説明"),
								OnCheckStateChanged: func() {
									changeSizingCheck(mWidgets.Window(), sizingState)
								},
							},
							declarative.CheckBox{
								AssignTo:    &sizingState.SizingShoulderCheck,
								Text:        mi18n.T("肩補正"),
								ToolTipText: mi18n.T("肩補正説明"),
								OnCheckStateChanged: func() {
									changeSizingCheck(mWidgets.Window(), sizingState)
								},
							},
							declarative.CheckBox{
								AssignTo:    &sizingState.SizingFingerStanceCheck,
								Text:        mi18n.T("指スタンス補正"),
								ToolTipText: mi18n.T("指スタンス補正説明"),
								OnCheckStateChanged: func() {
									changeSizingCheck(mWidgets.Window(), sizingState)
								},
							},
							declarative.CheckBox{
								AssignTo:    &sizingState.SizingArmTwistCheck,
								Text:        mi18n.T("捩り補正"),
								ToolTipText: mi18n.T("捩り補正説明"),
								OnCheckStateChanged: func() {
									changeSizingCheck(mWidgets.Window(), sizingState)
								},
							},
							declarative.CheckBox{
								AssignTo:    &sizingState.SizingWristCheck,
								Text:        mi18n.T("手首位置合わせ"),
								ToolTipText: mi18n.T("手首位置合わせ説明"),
								OnCheckStateChanged: func() {
									// changeSizingCheck(mWidgets.Window(), sizingState)
								},
							},
						},
					},
					declarative.Composite{
						Layout: declarative.Grid{Columns: 8},
						Children: []declarative.Widget{
							declarative.TextLabel{
								Text: mi18n.T("肩の比重"),
							},
							declarative.TextEdit{
								AssignTo: &sizingState.ShoulderWeightEdit,
								OnTextChanged: func() {
									if sizingState.ShoulderWeightEdit.Text() != "" {
										weight, err := strconv.Atoi(sizingState.ShoulderWeightEdit.Text())
										if err != nil {
											sizingState.ShoulderWeightSlider.SetValue(0)
											return
										}
										sizingState.ShoulderWeightSlider.SetValue(weight)
									}
								},
								MinSize: declarative.Size{Width: 30, Height: 20},
								MaxSize: declarative.Size{Width: 30, Height: 20},
							},
							declarative.TextLabel{
								Text: "%",
							},
							declarative.Slider{
								AssignTo:    &sizingState.ShoulderWeightSlider,
								ToolTipText: mi18n.T("肩比重説明"),
								OnValueChanged: func() {
									sizingState.ShoulderWeightEdit.ChangeText(
										strconv.Itoa(sizingState.ShoulderWeightSlider.Value()))
									changeSizingCheck(mWidgets.Window(), sizingState)
								},
								ColumnSpan: 5,
							},
						},
					},
					declarative.VSeparator{},
					declarative.Composite{
						Layout: declarative.Grid{Columns: 3},
						Children: []declarative.Widget{
							declarative.CheckBox{
								AssignTo:    &sizingState.AdoptSizingCheck,
								Text:        mi18n.T("即時反映"),
								ToolTipText: mi18n.T("即時反映説明"),
								Checked:     true,
							},
							declarative.CheckBox{
								AssignTo:    &sizingState.AdoptAllCheck,
								Text:        mi18n.T("全セット反映"),
								ToolTipText: mi18n.T("全セット反映説明"),
								Checked:     true,
							},
							sizingState.TerminateButton.Widgets(),
						},
					},
					declarative.VSeparator{},
					sizingState.OutputMotionPicker.Widgets(),
					sizingState.OutputModelPicker.Widgets(),
				},
			},
			sizingState.SaveButton.Widgets(),
			sizingState.Player.Widgets(),
		},
	}
}

func changeSizingCheck(cw *controller.ControlWindow, sizingState *domain.SizingState) {
	startIndex := 0
	endIndex := len(sizingState.SizingSets)
	if !sizingState.AdoptAllCheck.Checked() {
		// 現在のインデックスのみ
		startIndex = sizingState.CurrentIndex()
		endIndex = min(len(sizingState.SizingSets), startIndex+1)
	}

	for _, sizingSet := range sizingState.SizingSets[startIndex:endIndex] {
		sizingSet.IsSizingLeg = sizingState.SizingLegCheck.Checked()
		sizingSet.IsSizingUpper = sizingState.SizingUpperCheck.Checked()
		sizingSet.IsSizingShoulder = sizingState.SizingShoulderCheck.Checked()
		sizingSet.IsSizingArmStance = sizingState.SizingArmStanceCheck.Checked()
		sizingSet.IsSizingFingerStance = sizingState.SizingFingerStanceCheck.Checked()
		sizingSet.IsSizingArmTwist = sizingState.SizingArmTwistCheck.Checked()
		sizingSet.IsSizingWrist = sizingState.SizingWristCheck.Checked()
		sizingSet.ShoulderWeight = sizingState.ShoulderWeightSlider.Value()

		outputPath := sizingSet.CreateOutputMotionPath()
		if outputPath != "" {
			sizingSet.OutputMotionPath = outputPath
			if sizingSet.Index == sizingState.CurrentIndex() {
				sizingState.OutputMotionPicker.SetPath(outputPath)
			}
		}
	}

	if !sizingState.AdoptSizingCheck.Checked() {
		// 即時反映がOFFの場合は、サイジングを実行しない
		return
	}

	// エラーを受け取るためのチャネルを作成
	errCh := make(chan error, 1)

	// goroutineで処理を実行し、エラーをチャネル経由で返す
	go func() {
		err := usecase.ExecSizing(cw, sizingState)
		errCh <- err // エラーがnilでも送信
	}()

	// UIスレッドでエラーを受け取る
	go func() {
		if err := <-errCh; err != nil {
			// UIスレッドでエラーを表示する
			cw.Synchronize(func() {
				if ok := merr.ShowErrorDialog(cw.AppConfig(), err); ok {
					sizingState.SetSizingEnabled(true)
				}
			})
		}
	}()
}
