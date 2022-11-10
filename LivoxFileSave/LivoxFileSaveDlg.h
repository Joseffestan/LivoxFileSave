
// LivoxFileSaveDlg.h : 头文件
//

#pragma once
#include "afxwin.h"
#include <string>
#include <iostream>
#include <thread>
#include "Viewer3D.h"


// CLivoxFileSaveDlg 对话框
class CLivoxFileSaveDlg : public CDialogEx
{
// 构造
public:
	CLivoxFileSaveDlg(CWnd* pParent = NULL);	// 标准构造函数
	POINT pOldDlgSize;//存放主对话框大小

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_LIVOXFILESAVE_DIALOG };
#endif
	 
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;
	
	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	osg::ref_ptr<CViewer3D> m_spViewer3DRealTime;//view3D视图
	osg::ref_ptr<CViewer3D> m_spViewer3DLastFrame;
	CWinThread* m_WinThreadRealTime = NULL;
	CWinThread* m_WinThreadLastFrame = NULL;
	int m_nBtnRealTimeClicked = 0;//第n次点击显示实时点云按钮
	int m_nBtnLastFrameClicked = 0;
	afx_msg void OnBnClickedBtnStartSampling();
	afx_msg void OnBnClickedBtnChooseSavePath();
	afx_msg void OnBnClickedBtnShowTestData();
	CString m_strSavePath;// 提示消息
	afx_msg void OnSize(UINT nType, int cx, int cy);//控件随窗口尺寸变化
	int m_nSamplingDuration;// 采样持续时长（秒）
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnDestroy();
	afx_msg void OnBnClickedCancel();
	afx_msg void OnBnClickedBtnRealTimeData();
	
	afx_msg void OnStnClickedPctLastFrameData();
	afx_msg void OnStnClickedPctRealTimeData();
	afx_msg void OnEnChangeEdtSamplingDuration();
};
