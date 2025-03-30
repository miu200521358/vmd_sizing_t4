//go:build windows
// +build windows

package main

import (
	"embed"
	"fmt"
	"runtime"

	"miu200521358/vmd_sizing_t4/pkg/ui"

	"github.com/go-gl/glfw/v3.3/glfw"
	"github.com/miu200521358/walk/pkg/declarative"
	"github.com/miu200521358/walk/pkg/walk"

	"github.com/miu200521358/mlib_go/pkg/config/mconfig"
	"github.com/miu200521358/mlib_go/pkg/config/mi18n"
	"github.com/miu200521358/mlib_go/pkg/config/mproc"
	"github.com/miu200521358/mlib_go/pkg/domain/state"
	"github.com/miu200521358/mlib_go/pkg/interface/app"
	"github.com/miu200521358/mlib_go/pkg/interface/controller"
	"github.com/miu200521358/mlib_go/pkg/interface/viewer"
)

var env string

func init() {
	runtime.LockOSThread()

	mproc.SetMaxProcess(false)

	walk.AppendToWalkInit(func() {
		walk.MustRegisterWindowClass(controller.ConsoleViewClass)
	})
}

//go:embed app/*
var appFiles embed.FS

//go:embed i18n/*
var appI18nFiles embed.FS

func main() {
	viewerCount := 2
	appConfig := mconfig.LoadAppConfig(appFiles)
	appConfig.Env = env
	mi18n.Initialize(appI18nFiles)
	shared := state.NewSharedState(viewerCount)

	widths, heights, positionXs, positionYs := app.GetCenterSizeAndWidth(appConfig, viewerCount)

	var controlWindow *controller.ControlWindow
	viewerWindowList := viewer.NewViewerList(shared, appConfig)
	var err error

	go func() {
		// 操作ウィンドウは別スレッドで起動
		defer app.SafeExecute(appConfig, func() {
			widgets := &controller.MWidgets{
				Position: &walk.Point{X: positionXs[0], Y: positionYs[0]},
			}

			controlWindow, err = controller.NewControlWindow(shared, appConfig,
				ui.NewMenuItems(), []declarative.TabPage{ui.NewSizingPage(widgets)}, widgets.SetEnabledInPlaying,
				widths[0], heights[0], positionXs[0], positionYs[0])
			if err != nil {
				app.ShowErrorDialog(appConfig, err)
				return
			}

			widgets.SetWindow(controlWindow)
			widgets.OnLoaded()

			controlWindow.Run()
		})
	}()

	// GL初期化
	if err := glfw.Init(); err != nil {
		app.ShowErrorDialog(appConfig, fmt.Errorf("failed to initialize GLFW: %v", err))
		return
	}

	// 描画ウィンドウはメインスレッドで起動
	defer app.SafeExecute(appConfig, func() {
		for n := range viewerCount {
			nIdx := n + 1
			if err := viewerWindowList.Add("Viewer",
				widths[nIdx], heights[nIdx], positionXs[nIdx], positionYs[nIdx]); err != nil {
				app.ShowErrorDialog(appConfig, err)
				return
			}
		}

		viewerWindowList.InitOverride()
		viewerWindowList.Run()
	})
}
