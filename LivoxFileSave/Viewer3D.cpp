#include "stdafx.h"
#include "Viewer3D.h"
#include "osg/StateSet"
#include "osgDB/ImagePager"
#include "osgDB/Registry"
#include "osgDB/ReadFile"
#include "osgViewer/api/win32/GraphicsWindowWin32"
#include "osgGA/TrackballManipulator"
#include "osgGA/TerrainManipulator"
#include <math.h>
#include "ThreadSafeUpdateCallback.h"
#include "CameraManipulator.h"

//////////////////////////////////////////////////////////////////////////
// CViewerHandler
CViewerHandler::CViewerHandler()
{
}

CViewerHandler::~CViewerHandler()
{
}

bool CViewerHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{

    osg::ref_ptr<osgViewer::View> spView = dynamic_cast<osgViewer::View*>(&aa);
    if( spView == NULL || spView->getCamera() == NULL )
        return false;

    switch(ea.getEventType())
    {
    case osgGA::GUIEventAdapter::RESIZE:
    {
        setWindowSize(spView->getCamera(), ea.getWindowWidth(), ea.getWindowHeight());
    }
    break;

    default:
        break;
    }
    return false;
}

void CViewerHandler::setWindowSize(osg::Camera* pCamera, int width, int height)
{
    if( pCamera == NULL || width <= 0 || height <= 0 )
        return;

    pCamera->setViewport(0, 0, width, height);

    double fovy;
    double aspectRatio;
    double zNear;
    double zFar;
    if( pCamera->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar) )
        pCamera->setProjectionMatrixAsPerspective(fovy, (double)width / (double)height, zNear, zFar);
}
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//
CViewer3D::CViewer3D(void)
{
    m_hWnd = NULL;
    m_spViewer = NULL;
    m_spData = NULL;
    m_spKeyswitchManipulator = NULL;
    m_spStatesetManipulator = NULL;
    _firstFrame = true;
    //osgDB::Registry::instance()->setBuildKdTreesHint(osgDB::ReaderWriter::Options::BUILD_KDTREES);
    //osgDB::Registry::instance()->getOrCreateSharedStateManager();
}

CViewer3D::~CViewer3D(void)
{
    Destroy();
}

bool CViewer3D::ChangePosition(int nWidth, int nHeight)
{
/*
#ifdef _2D
#else
	Lock();
	compass->ChangePosition(nWidth, nHeight);
	Unlock();
#endif

    return true;
	*/
	return true;
}

void CViewer3D::InitManipulators(void)
{
    if( m_spKeyswitchManipulator != NULL )
        return;

    // Create a Manipulator Switcher
    m_spKeyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

    osg::ref_ptr<osgGA::CameraManipulator> defaultManipulator = new TrackballManipulatorEx(osgGA::StandardManipulator::UPDATE_MODEL_SIZE | osgGA::StandardManipulator::COMPUTE_HOME_USING_BBOX | osgGA::StandardManipulator::PROCESS_MOUSE_WHEEL);//new osgGA::TrackballManipulator();

    // Add our trackball manipulator to the switcher
    m_spKeyswitchManipulator->addMatrixManipulator( 0, "Default", defaultManipulator.get());
	//m_spKeyswitchManipulator->addMatrixManipulator( 1, "Default2", new osgGA::TrackballManipulator());

    // Init the switcher to the first manipulator (in this case the only manipulator)
    m_spKeyswitchManipulator->selectMatrixManipulator(0);  // Zero based index Value
}

void CViewer3D::UseSky(BOOL useSky)
{

}

bool CViewer3D::Create(size_t hWnd, osg::Node* node, int x, int y, int width, int height, osg::Vec4f color)
{
    if( hWnd == NULL )
        return false;

    Destroy();

    InitManipulators();

    m_spData = nullptr;

    osg::ref_ptr<osg::Group> spGroup = new osg::Group();
    //spGroup->addUpdateCallback(new CThreadSafeUpdateCallback);
    spGroup->setName("11");
	m_spData = spGroup;

    m_hWnd = (HWND)hWnd;
    // Local Variable to hold window size data
    RECT rect;

    // Create the viewer for this window
    m_spViewer = new osgViewer::Viewer();
	m_spViewer->setThreadingModel( osgViewer::Viewer::SingleThreaded);
	

	/*
	{
		osgViewer::ViewerBase::Scenes scenes;
		m_spViewer->getScenes(scenes, true);
		for(osgViewer::ViewerBase::Scenes::iterator itr = scenes.begin();
			itr != scenes.end();
			++itr)
		{
			osg::ref_ptr<osgDB::DatabasePager> spDatabasePager = (*itr)->getDatabasePager();
			if( spDatabasePager.valid() )
				spDatabasePager->setTargetMaximumNumberOfPageLOD(300);
		}
	}
	*/

	m_spViewer->setKeyEventSetsDone(0);
    
    m_spViewer->addEventHandler(new CViewerHandler);
    // Add a Stats Handler to the viewer
    //#ifdef _DEBUG
    m_spViewer->addEventHandler(new osgViewer::StatsHandler);
    //#endif
    // 		//	m_pViewer->addEventHandler(new osgViewer::WindowSizeHandler);
    // 		//	m_pViewer->addEventHandler(new osgViewer::HelpHandler);
    // 		// 	m_pViewer->addEventHandler(new osgViewer::LODScaleHandler);
    // 		// 	m_pViewer->addEventHandler(new osgViewer::RecordCameraPathHandler);
    // 		// 	m_pViewer->addEventHandler(new osgViewer::ScreenCaptureHandler);
    // 		// 	m_pViewer->addEventHandler(new osgViewer::ToggleSyncToVBlankHandler);
    // 		// 	m_pViewer->addEventHandler(new osgViewer::ThreadingHandler);
    //
    // 		m_spPickModelHandler = new CPickModelHandler;
    // 		m_spPickModelHandler->SetHwnd(hWnd);
    // 		m_pViewer->addEventHandler(m_spPickModelHandler.get());
    //
    // 		m_spPlaceModelHandler = new PlaceModelHandler;
    // 		m_pViewer->addEventHandler(m_spPlaceModelHandler.get());
    //


	if( 1 )
	{
		if( width == 0 || height == 0 )
		{
			// Get the current window size
			::GetWindowRect(m_hWnd, &rect);
			// 初始窗体大小不能为0
			//if( rect.right==rect.left )
			width = GetSystemMetrics(SM_CXSCREEN) + 2 * ::GetSystemMetrics(SM_CXBORDER);
			//else
			//width = rect.right - rect.left;

			//if( rect.top==rect.bottom )
			height = GetSystemMetrics(SM_CYSCREEN) + 2 * ::GetSystemMetrics(SM_CYBORDER);
			//else
			//height = rect.bottom - rect.top;
		}


		// Init the GraphicsContext Traits
		osg::ref_ptr<osg::GraphicsContext::Traits> spTraits = new osg::GraphicsContext::Traits;

		// Setup the traits parameters
		spTraits->readDISPLAY();
		if (spTraits->displayNum<0) spTraits->displayNum = 0;
		spTraits->screenNum = 0;
		spTraits->x = x;
		spTraits->y = y;
		spTraits->width = width;
		spTraits->height = height;
		spTraits->windowDecoration = false;
		spTraits->overrideRedirect = false;
		spTraits->doubleBuffer = true;
		spTraits->sharedContext = 0;
		spTraits->setInheritedWindowPixelFormat = true;

		// Init the Windata Variable that holds the handle for the Window to display OSG in.
		spTraits->inheritedWindowData = new osgViewer::GraphicsWindowWin32::WindowData(m_hWnd);
		//spTraits->quadBufferStereo = false;

		spTraits->alpha = osg::DisplaySettings::instance()->getMinimumNumAlphaBits();
		spTraits->stencil = osg::DisplaySettings::instance()->getMinimumNumStencilBits();
		spTraits->sampleBuffers = osg::DisplaySettings::instance()->getNumMultiSamples();
		spTraits->samples = osg::DisplaySettings::instance()->getNumMultiSamples();

		// Create the Graphics Context
		osg::ref_ptr<osg::GraphicsContext> spGraphicsContext = osg::GraphicsContext::createGraphicsContext(spTraits.get());

		// Init a new Camera (Master for this View)
		osg::ref_ptr<osg::Camera> spCamera = m_spViewer->getCamera();// new osg::Camera;

		// Assign Graphics Context to the Camera
		spCamera->setGraphicsContext(spGraphicsContext);

		// Set the viewport for the Camera
		spCamera->setViewport(new osg::Viewport(0, 0, spTraits->width, spTraits->height));
	}

	{
		osg::ref_ptr<osg::DisplaySettings> spDisplaySettings = m_spViewer->getDisplaySettings() ? m_spViewer->getDisplaySettings() : osg::DisplaySettings::instance().get();
		if( spDisplaySettings.valid() )
			spDisplaySettings->setStereo(true);
	}
	
	osg::ref_ptr<osg::Camera> spCamera = m_spViewer->getCamera();
    // Set projection matrix and camera attribtues
    //spCamera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    //spCamera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
    //spCamera->setClearColor(color);
    //spCamera->setProjectionMatrixAsPerspective(
    //	60.0f, static_cast<double>(spTraits->width) / static_cast<double>(spTraits->height), 1.0f, 10000.0f);
    //spCamera->setNearFarRatio( 0.0005 );
    //spCamera->setSmallFeatureCullingPixelSize(0);
    //spCamera->setComputeNearFarMode(osg::CullSettings::COMPUTE_NEAR_FAR_USING_PRIMITIVES);
    //spCamera->setNearFarRatio(0.00001f);

    //spCamera->setDepthSortImpostorSprites(true);
    //spCamera->setCullingMode(osg::Camera::ENABLE_ALL_CULLING & ~osg::Camera::SMALL_FEATURE_CULLING);
	double fovy, aspectRatio, zNear, zFar;
	spCamera->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
    spCamera->setProjectionMatrixAsPerspective( fovy, aspectRatio, 0.05f, 100000.0f);

	spCamera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    spCamera->setClearColor(color);	
	
    osg::ref_ptr<osg::StateSet> spStateset = spCamera->getOrCreateStateSet();
    spStateset->setGlobalDefaults();

    m_spStatesetManipulator  = new osgGA::StateSetManipulator( spStateset.get() );
    m_spStatesetManipulator->setKeyEventCyclePolygonMode(-1);
    m_spStatesetManipulator->setKeyEventToggleBackfaceCulling(-1);
    m_spStatesetManipulator->setKeyEventToggleLighting(-1);
    m_spStatesetManipulator->setKeyEventToggleTexturing(-1);
    m_spStatesetManipulator->setBackfaceEnabled(false);
    m_spStatesetManipulator->setTextureEnabled(true);
    m_spStatesetManipulator->setLightingEnabled(false);

    // Add the Camera to the Viewer
    //m_pViewer->addSlave(camera.get());
    //m_spViewer->setCamera(spCamera.get());

    // Add the Camera Manipulator to the Viewer
    m_spViewer->setCameraManipulator(m_spKeyswitchManipulator.get());

	//osgDB::Registry::instance()->setBuildKdTreesHint(osgDB::ReaderWriter::Options::BUILD_KDTREES);
	//if( osgDB::Registry::instance()->getKdTreeBuilder()==nullptr 
	//	|| dynamic_cast<KdTreeBuilderEx*>(osgDB::Registry::instance()->getKdTreeBuilder())==nullptr )
	//{
	//	osgDB::Registry::instance()->setKdTreeBuilder(new KdTreeBuilderEx());
	//}


    // Set the Scene Data
    m_spViewer->setSceneData(m_spData.get());

    // Realize the Viewer
    m_spViewer->realize();

    _firstFrame = true;
    StartRender();
    return true;
}

bool CViewer3D::Destroy()
{
    StopRender();

    if( m_spViewer != NULL )
    {
        m_spViewer->setDone(true);
        m_spViewer->stopThreading();
    }

    m_spStatesetManipulator = NULL;
    m_spKeyswitchManipulator = NULL;
    m_spViewer = NULL;
    m_spData = NULL;
    m_hWnd = NULL;
    return true;
}

void CViewer3D::StartRender()
{
    StopRender();

    if( m_spViewer != NULL )
    {
        m_spViewer->setDone(false);
        start();
    }
}

void CViewer3D::StopRender()
{
    if( !isRunning() )
        return;
    
    if( m_spViewer )
        m_spViewer->setDone(true);

	int nNum = 0;
    while( isRunning() )
	{
        OpenThreads::Thread::YieldCurrentThread();
		nNum++;
	}

	cancel();
}

void CViewer3D::Lock()
{
    _mutex.lock();
}

void CViewer3D::Unlock()
{
    _mutex.unlock();
}

void CViewer3D::run()
{
    if( m_spViewer == NULL )
        return;

    //m_spViewer->run();

    if (!m_spViewer->getCameraManipulator() && m_spViewer->getCamera()->getAllowEventFocus())
    {
        m_spViewer->setCameraManipulator(new osgGA::TrackballManipulator());
    }

    m_spViewer->setReleaseContextAtEndOfFrameHint(false);

    if (!m_spViewer->isRealized())
    {
        m_spViewer->realize();
    }

    const char* run_frame_count_str = getenv("OSG_RUN_FRAME_COUNT");
    unsigned int runTillFrameNumber = run_frame_count_str == 0 ? osg::UNINITIALIZED_FRAME_NUMBER : atoi(run_frame_count_str);

    while(!testCancel() && !m_spViewer->done() && (run_frame_count_str == 0 || m_spViewer->getViewerFrameStamp()->getFrameNumber() < runTillFrameNumber))
    {
        double minFrameTime = m_spViewer->getRunMaxFrameRate() > 0.0 ? 1.0 / m_spViewer->getRunMaxFrameRate() : 0.0;
        osg::Timer_t startFrameTick = osg::Timer::instance()->tick();

        if( testCancel() || m_spViewer->done() )
            break;

        if (m_spViewer->getRunFrameScheme() == osgViewer::ViewerBase::ON_DEMAND)
        {
            if (m_spViewer->checkNeedToDoFrame())
            {
                frame();
            }
            else
            {
                // we don't need to render a frame but we don't want to spin the run loop so make sure the minimum
                // loop time is 1/100th of second, if not otherwise set, so enabling the frame microSleep below to
                // avoid consume excessive CPU resources.
                if (minFrameTime == 0.0) minFrameTime = 0.01;
            }
        }
        else
        {
            frame();
        }

        if( testCancel() || m_spViewer->done() )
            break;

        // work out if we need to force a sleep to hold back the frame rate
        osg::Timer_t endFrameTick = osg::Timer::instance()->tick();
        double frameTime = osg::Timer::instance()->delta_s(startFrameTick, endFrameTick);
        if (frameTime < minFrameTime)
            OpenThreads::Thread::microSleep(static_cast<unsigned int>(1000000.0 * (minFrameTime - frameTime)));
    }
}

void CViewer3D::frame(double simulationTime)
{
    if( m_spViewer == NULL )
        return;

    if (m_spViewer->done())
        return;

	bool firstFrame = _firstFrame;
    if (_firstFrame)
    {
        m_spViewer->init();

        if (!m_spViewer->isRealized())
        {
            m_spViewer->realize();
        }

        _firstFrame = false;
    }
	
    m_spViewer->advance(simulationTime);
    m_spViewer->eventTraversal();

	Lock();
    m_spViewer->updateTraversal();    
    m_spViewer->renderingTraversals();
	if( firstFrame )
	{		
		m_spViewer->renderingTraversals();
		osg::ref_ptr<osg::DisplaySettings> spDisplaySettings = m_spViewer->getDisplaySettings() ? m_spViewer->getDisplaySettings() : osg::DisplaySettings::instance().get();
		if( spDisplaySettings.valid() )
			spDisplaySettings->setStereo(false);
	}
	Unlock();
}

osg::Matrix CViewer3D::matrixListtoSingle(const osg::MatrixList &tmplist)
{
    osg::Matrix tmp;

    if (tmplist.size() > 0)
    {
        unsigned int i;
        for (i = 1, tmp = tmplist[0]; i < tmplist.size(); i++)
            tmp *= tmplist[0];
        tmp = tmplist[0];
    }
    return (tmp);
}

bool CViewer3D::openFile(const std::string& file, osg::Node* pParentNode)
{
    osg::ref_ptr<osg::Node> spNode = osgDB::readNodeFile(file);
    if( !spNode.valid() )
        return false;

    return AddNode(spNode.get(), pParentNode);
}

bool CViewer3D::AddNode(osg::Node* pNode, osg::Node* pParentNode)
{
    if( pNode == NULL )
        return false;

    osg::ref_ptr<osg::Group> spGroup = NULL;
    if( pParentNode == NULL )
        spGroup = getSceneData()->asGroup();
    else
        spGroup = pParentNode->asGroup();

    if( spGroup == NULL )
        return false;

    bool bResetView = false;
    if( spGroup->getNumChildren() <= 0 )
        bResetView = true;

    osg::ref_ptr<CThreadSafeUpdateCallback> spCallback = dynamic_cast<CThreadSafeUpdateCallback*>(spGroup->getUpdateCallback());
    if( spCallback != NULL )
        spCallback->AddChild(spGroup.get(), pNode);
    else
    {
        Lock();
        if( !(spGroup->addChild(pNode)) )
        {
            Unlock();
            return false;
        }
        Unlock();
    }

    if( bResetView && m_spViewer != NULL )
    {
        //CenterNode(pNode, pNode->getBound().radius()*10.0);
		/*
        osg::ref_ptr<osgGA::GUIEventAdapter> spGUIEventAdapter = m_spViewer->getEventQueue()->createEvent();
        if( spGUIEventAdapter!=NULL )
		{
			//m_spViewer->getCameraManipulator()->setAutoComputeHomePosition(true);
			m_spViewer->getCameraManipulator()->home(*spGUIEventAdapter, *m_spViewer);
			//m_spViewer->getCameraManipulator()->setAutoComputeHomePosition(false);
		}
		*/
    }

    return true;
}

bool CViewer3D::RemoveNode(osg::Node* pNode, osg::Node* pParentNode)
{
    if( pNode == NULL )
        return false;

    osg::ref_ptr<osg::Group> spGroup = NULL;
    if( pParentNode == NULL )
    {
        if( pNode->getNumParents() > 0 )
            spGroup = pNode->getParent(0);
        else
            spGroup = getSceneData()->asGroup();
    }
    else
        spGroup = pParentNode->asGroup();

    if( spGroup == NULL )
        return false;

    bool bResetView = false;
    if( spGroup->getNumChildren() <= 0 )
        bResetView = true;

    osg::ref_ptr<CThreadSafeUpdateCallback> spCallback = dynamic_cast<CThreadSafeUpdateCallback*>(spGroup->getUpdateCallback());
    if( spCallback != NULL )
        spCallback->RemoveChild(spGroup.get(), pNode);
    else
    {
        Lock();
        if( !(spGroup->removeChild(pNode)) )
        {
            Unlock();
            return false;
        }
        Unlock();
    }    
    return true;
}

bool CViewer3D::RemoveChild(osg::Node* pNode, unsigned int pos, unsigned int numChildrenToRemove)
{
	if( pNode == NULL )
		return false;

	osg::ref_ptr<osg::Group> spGroup = pNode->asGroup();
	if( !spGroup.valid() || pos>=spGroup->getNumChildren())
		return false;

	if( (int)numChildrenToRemove<=0 )
		numChildrenToRemove = spGroup->getNumChildren() - pos;
	osg::ref_ptr<CThreadSafeUpdateCallback> spCallback = dynamic_cast<CThreadSafeUpdateCallback*>(pNode->getUpdateCallback());
	if( spCallback != NULL )
		spCallback->RemoveChild(pNode, pos, numChildrenToRemove);
	else
	{
		Lock();		
		if( !(spGroup->removeChildren(pos, numChildrenToRemove)) )
		{
			Unlock();
			return false;
		}
		Unlock();
	}	
	return true;
}

bool CViewer3D::CenterNode(osg::Node* pNode, double dMinDis)
{
    if (m_spKeyswitchManipulator == NULL || pNode == NULL )
        return false;

    osg::ref_ptr<osgGA::StandardManipulator> spCameraManipulator = dynamic_cast<osgGA::StandardManipulator*>(m_spKeyswitchManipulator->getCurrentMatrixManipulator());
    if( spCameraManipulator == NULL )
        return false;

    const osg::BoundingSphere& bs = pNode->getBound();
    if (fabs(bs.radius()) <= DBL_EPSILON) // invalid node
        return false;

    osg::Matrix mat = matrixListtoSingle( pNode->getWorldMatrices() );

    osg::Vec3d eye;
    osg::Vec3d center;
    osg::Vec3d up;
    spCameraManipulator->getTransformation(eye, center, up);
    osg::Vec3d newCenter = bs.center() * mat;
    osg::Vec3d newEye = eye - center;
    newEye.normalize();
    newEye *= fabs(bs.radius()*1.5) < dMinDis ? dMinDis : fabs(bs.radius()*1.5);
    newEye += newCenter;

	osg::ref_ptr<TrackballManipulatorEx> spTrackballManipulatorEx = dynamic_cast<TrackballManipulatorEx*>(spCameraManipulator.get());
	if( spTrackballManipulatorEx.valid() )
		spTrackballManipulatorEx->flyTo(newEye, newCenter, up, 0.5);
	else
		spCameraManipulator->setTransformation(newEye, newCenter, up);
    return true;
}

bool CViewer3D::SetCamera(const osg::Vec3& eye, const osg::Vec3& center, const osg::Vec3& up)
{
	if (m_spKeyswitchManipulator == NULL)
		return false;

	osg::ref_ptr<osgGA::StandardManipulator> spCameraManipulator = dynamic_cast<osgGA::StandardManipulator*>(m_spKeyswitchManipulator->getCurrentMatrixManipulator());
	if (spCameraManipulator == NULL)
		return false;

	osg::ref_ptr<TrackballManipulatorEx> spTrackballManipulatorEx = dynamic_cast<TrackballManipulatorEx*>(spCameraManipulator.get());
	if (spTrackballManipulatorEx.valid())
		spTrackballManipulatorEx->flyTo(eye, center, up, 0.5);
	else
		spCameraManipulator->setTransformation(eye, center, up);
	return true;
}

bool CViewer3D::setHomePosition(osg::Vec3d center, double radius)
{
	if (m_spKeyswitchManipulator == NULL )
		return false;

	osg::ref_ptr<osgGA::StandardManipulator> spCameraManipulator = dynamic_cast<osgGA::StandardManipulator*>(m_spKeyswitchManipulator->getCurrentMatrixManipulator());
	if( spCameraManipulator == NULL )
		return false;

	double dist = 3.5f * radius;
	osg::Vec3d eye = center  + osg::Vec3d(0.0,-dist,0.0f);
	osg::Vec3d up(0.0, 0.0, 1.0);

	bool autoComputeHomePosition = spCameraManipulator->getAutoComputeHomePosition();
	spCameraManipulator->setHomePosition(eye, center, up);
	spCameraManipulator->home(0.0);
	spCameraManipulator->setAutoComputeHomePosition(autoComputeHomePosition);
	return true;
}

bool CViewer3D::HighlightNode(osg::Node* pNode, bool bHighlight)
{
    return false;
}

void CViewer3D::Clear()
{
	if( !m_spData.valid() || m_spData->getNumChildren()<=0U )
		return;

	Lock();
	m_spData->removeChildren(0U, m_spData->getNumChildren());
	Unlock();
}