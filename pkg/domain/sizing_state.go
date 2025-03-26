package domain

import (
	"fmt"

	"github.com/miu200521358/walk/pkg/walk"
)

type SizingState struct {
	NavToolBar                  *walk.ToolBar // セットツールバー
	CurrentIndex                int           // 現在のインデックス
	SizingSets                  []*SizingSet  // サイジングセットリスト
	currentPageChangedPublisher walk.EventPublisher
}

func (ss *SizingState) AddSet() {
	action := ss.NewSet()
	ss.NavToolBar.Actions().Add(action)
	ss.setCurrentAction(len(ss.SizingSets) - 1)
}

func (ss *SizingState) NewSet() *walk.Action {
	index := len(ss.SizingSets)

	action := walk.NewAction()
	action.SetCheckable(true)
	action.SetExclusive(true)
	action.SetText(fmt.Sprintf(" No. %d ", index+1))

	action.Triggered().Attach(func() {
		ss.setCurrentAction(index)
	})

	ss.SizingSets = append(ss.SizingSets, NewSizingSet(index))

	return action
}

func (ss *SizingState) setCurrentAction(index int) {
	// 一旦すべてのチェックを外す
	for i := range len(ss.SizingSets) {
		ss.NavToolBar.Actions().At(i).SetChecked(false)
	}

	// 該当INDEXのみチェックON
	ss.CurrentIndex = index
	ss.NavToolBar.Actions().At(index).SetChecked(true)
	ss.currentPageChangedPublisher.Publish()
}

type SizingSet struct {
	Index int
}

func NewSizingSet(index int) *SizingSet {
	return &SizingSet{
		Index: index,
	}
}
