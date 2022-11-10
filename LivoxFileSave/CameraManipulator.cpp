#include "stdafx.h"
#include "CameraManipulator.h"
#include "osg/ComputeBoundsVisitor"

TrackballManipulatorEx::TrackballManipulatorEx(int flags) : osgGA::TrackballManipulator(flags)
{
	_flyTo = false;
}

bool TrackballManipulatorEx::performMovementLeftMouseButton( const double eventTimeDelta, const double dx, const double dy )
{
	if( 1 )
		return osgGA::TrackballManipulator::performMovementMiddleMouseButton(eventTimeDelta, dx, dy);
	else
	{
		bool bOld = getVerticalAxisFixed();
		setVerticalAxisFixed( true );
		bool bResult = osgGA::OrbitManipulator::performMovementLeftMouseButton(eventTimeDelta, dx, dy);
		setVerticalAxisFixed(bOld);
		return bResult;
	}
}

bool TrackballManipulatorEx::performMovementRightMouseButton( const double eventTimeDelta, const double dx, const double dy )
{
	if( 0 )
		return osgGA::TrackballManipulator::performMovementMiddleMouseButton(eventTimeDelta, dx, dy);
	else
	{
		bool bOld = getVerticalAxisFixed();
		setVerticalAxisFixed( true );
		bool bResult = osgGA::TrackballManipulator::performMovementLeftMouseButton(eventTimeDelta, dx, dy);
		setVerticalAxisFixed(bOld);
		return bResult;
	}	
}

void TrackballManipulatorEx::setMinimumDistance( const double& minimumDistance, bool relativeToModelSize )
{
	osgGA::TrackballManipulator::setMinimumDistance(minimumDistance, relativeToModelSize);
}

void TrackballManipulatorEx::setNode( osg::Node* node )
{
	osgGA::TrackballManipulator::setNode( node );

	// update model size
	if( _flags & UPDATE_MODEL_SIZE )
	{
		if( _node.valid() )
		{
			setMinimumDistance( osg::clampBetween( _modelSize * 0.001, 0.00001, 1.0 ) );
			OSG_INFO << "TrackballManipulatorEx: setting _minimumDistance to "
				<< _minimumDistance << std::endl;
		}
	}
}

void TrackballManipulatorEx::home( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
{
	if( getAutoComputeHomePosition() )
	{
		const osg::Camera *camera = us.asView() ? us.asView()->getCamera() : NULL;
		computeHomePosition( camera, ( _flags & COMPUTE_HOME_USING_BBOX ) != 0 );
	}

	_thrown = false;
	setTransformation( _homeEye, _homeCenter, _homeUp );

	us.requestRedraw();
	us.requestContinuousUpdate( false );
	flushMouseEventStack();
}

void TrackballManipulatorEx::home( double /*currentTime*/ )
{
	if( getAutoComputeHomePosition() )
		computeHomePosition( NULL, ( _flags & COMPUTE_HOME_USING_BBOX ) != 0 );

	_thrown = false;
	setTransformation( _homeEye, _homeCenter, _homeUp );
	flushMouseEventStack();
}

void TrackballManipulatorEx::computeHomePosition(const osg::Camera *camera, bool useBoundingBox)
{
	if (getNode())
	{
		osg::BoundingSphere boundingSphere;

		OSG_INFO<<" CameraManipulator::computeHomePosition("<<camera<<", "<<useBoundingBox<<")"<<std::endl;

		if (useBoundingBox)
		{
			// compute bounding box
			// (bounding box computes model center more precisely than bounding sphere)
			osg::ComputeBoundsVisitor cbVisitor;
			getNode()->accept(cbVisitor);
			osg::BoundingBox &bb = cbVisitor.getBoundingBox();

			if (bb.valid()) boundingSphere.expandBy(bb);
			else boundingSphere = getNode()->getBound();
		}
		else
		{
			// compute bounding sphere
			boundingSphere = getNode()->getBound();
		}

		// set dist to default
		double dist = /*3.5f * */boundingSphere.radius();

		if (camera)
		{

			// try to compute dist from frustum
			double left,right,bottom,top,zNear,zFar;
			if (camera->getProjectionMatrixAsFrustum(left,right,bottom,top,zNear,zFar))
			{
				double vertical2 = fabs(right - left) / zNear / 2.;
				double horizontal2 = fabs(top - bottom) / zNear / 2.;
				double dim = horizontal2 < vertical2 ? horizontal2 : vertical2;
				double viewAngle = atan2(dim,1.);
				dist = boundingSphere.radius() / sin(viewAngle);
			}
			else
			{
				// try to compute dist from ortho
				if (camera->getProjectionMatrixAsOrtho(left,right,bottom,top,zNear,zFar))
				{
					dist = fabs(zFar - zNear) / 2.;
				}
			}
		}

		// set home position
		setHomePosition(boundingSphere.center() + osg::Vec3d(0.0,-dist,0.0f),
			boundingSphere.center(),
			osg::Vec3d(0.0f,0.0f,1.0f),
			_autoComputeHomePosition);
		_flyTo = false;
	}
}

bool TrackballManipulatorEx::handleFrame( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
{
	bool result = osgGA::TrackballManipulator::handleFrame( ea, us );

	if( _flyTo )
	{
		_elapseTime += _delta_frame_time;
		if( _elapseTime>=_totalTime )
		{
			_elapseTime = _totalTime;
			_flyTo = false;
		}

		double centerDis = _vtCenter.length();
		double eyeDis = _vtEye.length();
		double upDis = _vtUp.length();
		osg::Vec3d vtCenter = _vtCenter;
		vtCenter.normalize();
		osg::Vec3d vtEye = _vtEye;
		vtEye.normalize();
		osg::Vec3d vtUp = _vtUp;
		vtUp.normalize();
		osg::Vec3d tmpCenter = _oldCenter + vtCenter*centerDis*_elapseTime/_totalTime;// * mat;
		osg::Vec3d tmpEye = _oldEye + vtEye*eyeDis*_elapseTime/_totalTime;
		osg::Vec3d tmpUp = _oldUp + vtUp*upDis*_elapseTime/_totalTime;
		setTransformation(tmpEye, tmpCenter, tmpUp);
		us.requestRedraw();
	}
	
	return result;
}

void TrackballManipulatorEx::flyTo(const osg::Vec3d& newEye, const osg::Vec3d& newCenter, const osg::Vec3d& newUp, double time)
{	
	getTransformation(_oldEye, _oldCenter, _oldUp);
	_vtCenter = newCenter - _oldCenter;
	_vtEye = newEye - _oldEye;
	_vtUp = newUp - _oldUp;
	_totalTime = time;
	_elapseTime = 0;
	_flyTo = true;
}