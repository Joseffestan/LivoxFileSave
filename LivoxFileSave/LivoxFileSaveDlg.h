
// LivoxFileSaveDlg.h : ͷ�ļ�
//

#pragma once
#include "afxwin.h"
#include <string>
#include <iostream>
#include <thread>
#include "Viewer3D.h"


// CLivoxFileSaveDlg �Ի���
class CLivoxFileSaveDlg : public CDialogEx
{
// ����
public:
	CLivoxFileSaveDlg(CWnd* pParent = NULL);	// ��׼���캯��
	POINT pOldDlgSize;//������Ի����С

// �Ի�������
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_LIVOXFILESAVE_DIALOG };
#endif
	 
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV ֧��


// ʵ��
protected:
	HICON m_hIcon;
	
	// ���ɵ���Ϣӳ�亯��
	virtual BOOL OnInitDialog();
	
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	osg::ref_ptr<CViewer3D> m_spViewer3DRealTime;//view3D��ͼ
	osg::ref_ptr<CViewer3D> m_spViewer3DLastFrame;
	CWinThread* m_WinThreadRealTime = NULL;
	CWinThread* m_WinThreadLastFrame = NULL;
	int m_nBtnRealTimeClicked = 0;//��n�ε����ʾʵʱ���ư�ť
	int m_nBtnLastFrameClicked = 0;
	afx_msg void OnBnClickedBtnStartSampling();
	afx_msg void OnBnClickedBtnChooseSavePath();
	afx_msg void OnBnClickedBtnShowTestData();
	CString m_strSavePath;// ��ʾ��Ϣ
	afx_msg void OnSize(UINT nType, int cx, int cy);//�ؼ��洰�ڳߴ�仯
	int m_nSamplingDuration;// ��������ʱ�����룩
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnDestroy();
	afx_msg void OnBnClickedCancel();
	afx_msg void OnBnClickedBtnRealTimeData();
	
	afx_msg void OnStnClickedPctLastFrameData();
	afx_msg void OnStnClickedPctRealTimeData();
	afx_msg void OnEnChangeEdtSamplingDuration();
};
