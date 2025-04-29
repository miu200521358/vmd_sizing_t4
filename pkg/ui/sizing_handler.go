package ui

import (
	"miu200521358/vmd_sizing_t4/pkg/domain"
	"miu200521358/vmd_sizing_t4/pkg/usecase"
	"sync"
	"sync/atomic"
	"time"

	"github.com/miu200521358/mlib_go/pkg/config/merr"
	"github.com/miu200521358/mlib_go/pkg/config/mi18n"
	"github.com/miu200521358/mlib_go/pkg/config/mlog"
	"github.com/miu200521358/mlib_go/pkg/interface/controller"
)

func changeSizingCheck(cw *controller.ControlWindow, sizingState *SizingState) {
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
		err := execSizing(cw, sizingState)
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

func execSizing(cw *controller.ControlWindow, sizingState *SizingState) error {
	if !sizingState.AdoptSizingCheck.Checked() ||
		(sizingState.CurrentSet().OriginalModel == nil &&
			sizingState.CurrentSet().SizingConfigModel == nil &&
			sizingState.CurrentSet().OutputMotion == nil) {
		return nil
	}

	for _, sizingSet := range sizingState.SizingSets {
		if !sizingSet.IsSizingLeg && sizingSet.CompletedSizingLeg ||
			!sizingSet.IsSizingUpper && sizingSet.CompletedSizingUpper ||
			!sizingSet.IsSizingShoulder && sizingSet.CompletedSizingShoulder ||
			!sizingSet.IsSizingArmStance && sizingSet.CompletedSizingArmStance ||
			!sizingSet.IsSizingFingerStance && sizingSet.CompletedSizingFingerStance ||
			!sizingSet.IsSizingArmTwist && sizingSet.CompletedSizingArmTwist ||
			!sizingSet.IsSizingWrist && sizingSet.CompletedSizingWrist ||
			sizingSet.ShoulderWeight != sizingSet.CompletedShoulderWeight {

			// チェックを外したら読み直し
			sizingSet.CompletedSizingLeg = false
			sizingSet.CompletedSizingUpper = false
			sizingSet.CompletedSizingShoulder = false
			sizingSet.CompletedSizingArmStance = false
			sizingSet.CompletedSizingFingerStance = false
			sizingSet.CompletedSizingArmTwist = false
			sizingSet.CompletedSizingWrist = false

			// オリジナルモーションをサイジング先モーションとして読み直し
			sizingState.SetCurrentIndex(sizingSet.Index)
			sizingState.LoadSizingMotion(cw, sizingSet.OriginalMotionPath, false)
		}
	}

	var completedProcessCount int32 = 0
	totalProcessCount := 0
	for _, sizingSet := range sizingState.SizingSets {
		totalProcessCount += sizingSet.GetProcessCount()
	}

	cw.Synchronize(func() {
		sizingState.SetSizingEnabled(false)
		sizingState.TerminateButton.SetEnabled(true)

		cw.ProgressBar().SetMax(totalProcessCount)
		cw.ProgressBar().SetValue(int(completedProcessCount))
	})

	// 処理時間の計測開始
	start := time.Now()

	scales := usecase.GenerateSizingScales(sizingState.SizingSets)
	isExec := false

	errorChan := make(chan error, len(sizingState.SizingSets))

	mlog.IL(mi18n.T("サイジング開始"))

	var wg sync.WaitGroup
	for _, sizingSet := range sizingState.SizingSets {
		if sizingSet.OriginalConfigModel == nil || sizingSet.SizingConfigModel == nil ||
			sizingSet.OutputMotion == nil {
			continue
		}

		wg.Add(1)
		go func(sizingSet *domain.SizingSet) {
			defer wg.Done()

			incrementCompletedCount := func() {
				atomic.AddInt32(&completedProcessCount, 1)
				cw.Synchronize(func() {
					cw.ProgressBar().Increment()
				})
			}

			// 腕指スタンス補正
			if execResult, err := usecase.NewSizingArmStanceUsecase().Exec(sizingSet, len(sizingState.SizingSets), incrementCompletedCount); err != nil {
				errorChan <- err
				return
			} else {
				if sizingSet.IsTerminate {
					isExec = false
					return
				}

				isExec = execResult || isExec

				if isExec {
					sizingSet.OutputMotion.SetRandHash()
					sizingSet.OutputMotion.SetName(sizingSet.SizingModel.Name())
					cw.StoreMotion(0, sizingSet.Index, sizingSet.OutputMotion)
				}
			}

			// 下半身・足補正
			if execResult, err := usecase.NewSizingLegUsecase().Exec(sizingSet, scales[sizingSet.Index], len(sizingState.SizingSets), incrementCompletedCount); err != nil {
				errorChan <- err
				return
			} else {
				if sizingSet.IsTerminate {
					isExec = false
					return
				}

				isExec = execResult || isExec

				if isExec {
					sizingSet.OutputMotion.SetRandHash()
					sizingSet.OutputMotion.SetName(sizingSet.SizingModel.Name())
					cw.StoreMotion(0, sizingSet.Index, sizingSet.OutputMotion)
				}
			}

			for _, uc := range []usecase.ISizingUsecase{
				usecase.NewSizingUpperUsecase(),    // 上半身補正
				usecase.NewSizingShoulderUsecase(), // 肩補正
			} {
				if execResult, err := uc.Exec(sizingSet, len(sizingState.SizingSets), incrementCompletedCount); err != nil {
					errorChan <- err
					return
				} else {
					if sizingSet.IsTerminate {
						isExec = false
						return
					}

					isExec = execResult || isExec

					if isExec {
						sizingSet.OutputMotion.SetRandHash()
						sizingSet.OutputMotion.SetName(sizingSet.SizingModel.Name())
						cw.StoreMotion(0, sizingSet.Index, sizingSet.OutputMotion)
					}
				}
			}

		}(sizingSet)
	}

	wg.Wait()
	close(errorChan)

	// チャネルからエラーを受け取る
	for err := range errorChan {
		if err != nil {
			if merr.IsTerminateError(err) {
				mlog.I(mi18n.T("サイジング中断"))
				break
			} else {
				mlog.E(mi18n.T("サイジングエラー", map[string]interface{}{
					"AppName": cw.AppConfig().Name, "AppVersion": cw.AppConfig().Version}), err, "")
				return err
			}
		}
	}

	// 処理時間の計測終了
	elapsed := time.Since(start)

	if isExec {
		mlog.ILT(mi18n.T("サイジング終了"), mi18n.T("サイジング終了メッセージ",
			map[string]interface{}{"ProcessTime": controller.FormatDuration(elapsed)}))
	} else {
		mlog.I(mi18n.T("サイジング終了"))
	}

	// 中断したら、データを戻してフラグを落としておく
	for _, sizingSet := range sizingState.SizingSets {
		if sizingSet.IsTerminate {
			// オリジナルモーションをサイジング先モーションとして読み直し
			outputMotion := cw.LoadMotion(1, sizingSet.Index)
			outputMotion.SetRandHash()
			cw.StoreMotion(0, sizingSet.Index, outputMotion)

			sizingSet.IsTerminate = false
		}
	}

	cw.Synchronize(func() {
		sizingState.SetSizingEnabled(true)
		sizingState.TerminateButton.SetEnabled(false)

		cw.ProgressBar().SetMax(0)
		cw.ProgressBar().SetValue(0)
	})

	// 最初に戻す(読み直しとかでINDEXがズレた時用)
	sizingState.ChangeCurrentAction(0)
	controller.Beep()

	return nil
}
