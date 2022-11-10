#include "stdafx.h"
#include "ThreadSafeUpdateCallback.h"

CThreadSafeUpdateCallback::CThreadSafeUpdateCallback(void)
{
}


CThreadSafeUpdateCallback::~CThreadSafeUpdateCallback(void)
{
}

void CThreadSafeUpdateCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
	// note, callback is responsible for scenegraph traversal so
	// they must call traverse(node,nv) to ensure that the
	// scene graph subtree (and associated callbacks) are traversed.
	osg::ref_ptr<osg::Group> spGroup = node->asGroup();
	if( spGroup!=NULL )
	{
		DWORD_PTR key = (DWORD_PTR)node;
		osg::ref_ptr<osg::Group> spChilds = NULL;

		//////////////////////////////////////////////////////////////////////////
		// Remove Child
		if(m_mapRemoveChilds.size()>0)
		{
			OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_removeMutex);
			MAP_NODE::iterator it = m_mapRemoveChilds.find(key);
			if( it!=m_mapRemoveChilds.end() )
			{
				spChilds = it->second;
				m_mapRemoveChilds.erase(it);		
			}
		}

		if( spChilds.valid() )
		{
			for( int i=0; i<spChilds->getNumChildren(); i++ )
			{
				osg::ref_ptr<osg::Node> spNode = spChilds->getChild(i);
				if( spNode==NULL )
					continue;

				spGroup->removeChild(spNode.get());
			}

			spChilds = NULL;
		}
		//////////////////////////////////////////////////////////////////////////

		//////////////////////////////////////////////////////////////////////////
		// Add Child
		{
			OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_addMutex);
			MAP_NODE::iterator it = m_mapAddChilds.find(key);
			if( it!=m_mapAddChilds.end() )
			{
				spChilds = it->second;
				m_mapAddChilds.erase(it);		
			}
		}
		
		if( spChilds.valid() )
		{
			for( int i=0; i<spChilds->getNumChildren(); i++ )
			{
				osg::ref_ptr<osg::Node> spNode = spChilds->getChild(i);
				if( spNode==NULL )
					continue;

				spGroup->addChild(spNode.get());
			}

			spChilds = NULL;
		}
		//////////////////////////////////////////////////////////////////////////
	}
	
	traverse(node,nv);
}

bool CThreadSafeUpdateCallback::hasThreadSafeUpdateCallback(osg::Callback* pCallback)
{
	if (pCallback == NULL)
		return false;

	osg::ref_ptr<osg::NodeCallback> spNodeCallback = dynamic_cast<osg::NodeCallback*>(pCallback);
	if (!spNodeCallback.valid())
		return false;

	osg::ref_ptr<CThreadSafeUpdateCallback> spThreadSafeUpdateCallback = dynamic_cast<CThreadSafeUpdateCallback*>(spNodeCallback.get());
	if (spThreadSafeUpdateCallback.valid())
		return true;

	return hasThreadSafeUpdateCallback(spNodeCallback->getNestedCallback());
}

bool CThreadSafeUpdateCallback::AddChild(osg::Node* parent, osg::Node* node)
{
	if( parent==NULL || node==NULL )
		return false;

	if( !hasThreadSafeUpdateCallback(node->getUpdateCallback()) )
		node->addUpdateCallback(this);

	DWORD_PTR key = (DWORD_PTR)parent;
	if( m_mapRemoveChilds.size()>0)
	{
		OpenThreads::ScopedLock<OpenThreads::Mutex>flock(_removeMutex);
		MAP_NODE::iterator it = m_mapRemoveChilds.find(key);
		if (it != m_mapRemoveChilds.end())
		{
			osg::ref_ptr<osg::Group> spGroup = it->second;
			if (spGroup.valid())
				spGroup->removeChild(node);
		}
	}

    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_addMutex);
    {
        osg::Group* pGroup = dynamic_cast<osg::Group*>(parent);
        if( pGroup!=nullptr )
        {
            if( pGroup->containsNode(node) )
                return false;

            if( pGroup->getNumParents()<=0 )
            {
                pGroup->addChild(node);
                return true;
            }
        }
    }
	MAP_NODE::iterator it = m_mapAddChilds.find(key);
	osg::ref_ptr<osg::Group> spGroup = NULL;
	if (it != m_mapAddChilds.end())	
		spGroup = it->second;
	else
	{
		spGroup = new osg::Group();
		m_mapAddChilds[key] = spGroup;
		if (!hasThreadSafeUpdateCallback(parent->getUpdateCallback()))
			parent->addUpdateCallback(this);
	}

	if( spGroup->containsNode(node) )
		return false;

	spGroup->addChild(node);
	return true;
}

bool CThreadSafeUpdateCallback::RemoveChild(osg::Node* parent, osg::Node* node)
{
	if( parent==NULL || node==NULL )
		return false;

	DWORD_PTR key = (DWORD_PTR)parent;
	{
		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_addMutex);	 
		MAP_NODE::iterator it = m_mapAddChilds.find(key);		 
		if( it!=m_mapAddChilds.end() )
		{
			osg::ref_ptr<osg::Group> spGroup = it->second;
			if( spGroup.valid() )			
				spGroup->removeChild(node);			
		}
	}

	OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_removeMutex);	 
    {
        osg::Group* pGroup = dynamic_cast<osg::Group*>(parent);
        if( pGroup!=nullptr )
        {
            if( !pGroup->containsNode(node) )
                return false;

            if( pGroup->getNumParents()<=0 )
            {
                pGroup->removeChild(node);
                return true;
            }
        }
    }
	MAP_NODE::iterator it = m_mapRemoveChilds.find(key);
	osg::ref_ptr<osg::Group> spGroup = NULL;
	if( it!=m_mapRemoveChilds.end() )
		spGroup = it->second;
	else
	{
		spGroup = new osg::Group();
		m_mapRemoveChilds[key] = spGroup;
		if (!hasThreadSafeUpdateCallback(parent->getUpdateCallback()))
			parent->addUpdateCallback(this);
	}

	if( spGroup->containsNode(node) )
		return false;

	spGroup->addChild(node);
	return true;
}


bool CThreadSafeUpdateCallback::RemoveChild(osg::Node* node, unsigned int pos, unsigned int numChildrenToRemove)
{
    if( node==nullptr || pos<0U || numChildrenToRemove<=0U )
        return false;

	DWORD_PTR key = (DWORD_PTR)node;
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_addMutex);
        MAP_NODE::iterator it = m_mapAddChilds.find(key);
        if( it!=m_mapAddChilds.end() )
        {
            osg::ref_ptr<osg::Group> spGroup = it->second;
            if( spGroup.valid() )
            {
                osg::Group* pGroup = dynamic_cast<osg::Group*>(node);
                if( pGroup!=nullptr )
                {
                    for(unsigned int i=pos; i<pos+numChildrenToRemove && i<pGroup->getNumChildren(); i++)
                    {
                        spGroup->removeChild(pGroup->getChild(i));
                    }
                }
            }
        }
    }

    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_removeMutex);
    {
        osg::Group* pGroup = dynamic_cast<osg::Group*>(node);
        if( pGroup!=nullptr )
        {
            if( pGroup->getNumParents()<=0 )
            {
                pGroup->removeChild(pos, numChildrenToRemove);
                return true;
            }
        }
    }
    MAP_NODE::iterator it = m_mapRemoveChilds.find(key);
    osg::ref_ptr<osg::Group> spGroup = NULL;
    if( it!=m_mapRemoveChilds.end() )
        spGroup = it->second;
    else
    {
        spGroup = new osg::Group();
        m_mapRemoveChilds[key] = spGroup;
        if (!hasThreadSafeUpdateCallback(node->getUpdateCallback()))
            node->addUpdateCallback(this);
    }

    osg::Group* pGroup = dynamic_cast<osg::Group*>(node);
    if( pGroup!=nullptr )
    {
        for(unsigned int i=pos; i<pos+numChildrenToRemove && i<pGroup->getNumChildren(); i++)
        {
            if( !spGroup->containsNode(pGroup->getChild(i)) )
                spGroup->addChild(pGroup->getChild(i));
        }
    }
    return true;
}