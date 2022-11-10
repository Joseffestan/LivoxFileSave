#pragma once
#include "osgViewer/Viewer"
#include "osgViewer/ViewerEventHandlers"
#include "osgGA/KeySwitchMatrixManipulator"
#include "osgGA/StateSetManipulator"
#include "osgText/Text"
#include "osgGA/StateSetManipulator"

#if defined(_MSC_VER) && _MSC_VER > 1600    // VC8/9
	#include <unordered_map>
	#define hash_multimap std::unordered_multimap
	#define hash_map std::unordered_map
#elif defined(_MSC_VER) && _MSC_VER > 1200    // VC8/9
	#include <hash_map>
	using stdext::hash_multimap;
	using stdext::hash_map;
#else                                       // VC6, GCC or others
	#define hash_multimap std::multimap
	#define hash_map std::map
#endif

class CViewerHandler : public osgGA::GUIEventHandler
{
public:
	CViewerHandler();
	virtual ~CViewerHandler();
	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

protected:
	void setWindowSize(osg::Camera* pCamera, int width, int height);
};

class CViewer3D : public OpenThreads::Thread, public osg::Referenced
{
public:
	CViewer3D(void);
	~CViewer3D(void);

public:	
	bool Create(size_t hWnd, osg::Node* node = NULL, int x = 0, int y = 0, int width = 0, int height = 0, osg::Vec4f color = osg::Vec4f(0.0f, 0.0f, 0.0f, 1.0f));
	virtual bool Destroy();

	virtual osgViewer::Viewer* getViewer() { return m_spViewer.get(); }
	virtual osg::Node* getRootNode() { if( m_spViewer==NULL ) return NULL; return m_spViewer->getSceneData(); }
	virtual osg::Node* getSceneData() { return m_spData.get(); }

	virtual bool openFile(const std::string& file, osg::Node* pParentNode = NULL);
	virtual bool AddNode(osg::Node* pNode, osg::Node* pParentNode = NULL);
	virtual bool RemoveNode(osg::Node* pNode, osg::Node* pParentNode = NULL);
	virtual bool RemoveChild(osg::Node* node, unsigned int pos, unsigned int numChildrenToRemove=1);
	virtual bool CenterNode(osg::Node* pNode, double dMinDis);	
	virtual bool SetCamera(const osg::Vec3& eye, const osg::Vec3& center, const osg::Vec3& up);
	virtual bool HighlightNode(osg::Node* pNode, bool bHighlight);
	virtual bool ChangePosition(int nWidth, int nHeight);	
	virtual osgGA::StateSetManipulator* getStateSetManipulator() { return m_spStatesetManipulator.get(); }

	bool setHomePosition(osg::Vec3d center, double radius);

public:	
	virtual void StartRender();
	virtual void StopRender();
	virtual void Lock();
	virtual void Unlock();
	virtual void Clear();

protected:
	osg::Matrix matrixListtoSingle(const osg::MatrixList &tmplist);

private:
	void InitManipulators(void);
	virtual void run();	
	virtual void frame(double simulationTime=USE_REFERENCE_TIME);
	OpenThreads::Mutex _mutex;
	bool _firstFrame;

public:
	void UseSky(BOOL useSky);

protected:
	HWND m_hWnd;
	osg::ref_ptr<osgViewer::Viewer> m_spViewer;
	osg::ref_ptr<osg::Group> m_spData;
	osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> m_spKeyswitchManipulator;
	osg::ref_ptr<osgGA::StateSetManipulator> m_spStatesetManipulator;
};


