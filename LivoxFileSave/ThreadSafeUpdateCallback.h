#pragma once
#include "osg/NodeCallback"
#include "osg/Group"
#include <map>

class CThreadSafeUpdateCallback : public osg::NodeCallback
{
public:
	CThreadSafeUpdateCallback(void);
	~CThreadSafeUpdateCallback(void);

public:
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);

public:
	bool AddChild(osg::Node* parent, osg::Node* node);
	bool RemoveChild(osg::Node* parent, osg::Node* node);
    bool RemoveChild(osg::Node* node, unsigned int pos, unsigned int numChildrenToRemove=1);

protected:
	bool hasThreadSafeUpdateCallback(osg::Callback* pCallback);

protected:
	typedef std::map<DWORD_PTR, osg::ref_ptr<osg::Group>> MAP_NODE;
	MAP_NODE m_mapAddChilds;
	MAP_NODE m_mapRemoveChilds;
	OpenThreads::Mutex _addMutex;
	OpenThreads::Mutex _removeMutex;
};