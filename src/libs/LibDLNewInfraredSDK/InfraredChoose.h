#pragma once

#include <QWidget>
#include <common/Singleton.hpp>
#include "InfraredBase.h"
#include "GuiDeNewInfrared.h"
#include "JuGeNewInfrared.h"

class InfraredChoose : public Singleton<InfraredChoose>
{
public:
    InfraredChoose() {}
    ~InfraredChoose() { releaseInfrared(); }

public:
	InfraredBase* getInfraredObject()
	{
		if (m_base == NULL && m_infraredType == SDK_GUIDE_INFRARED)
		{
            m_base = new GuiDeNewInfrared;
		}

        if (m_base == NULL && m_infraredType == SDK_JUGE_INFRARED)
        {
            m_base = new JuGeNewInfrared;
        }
		return m_base;
	}

    void releaseInfrared()
    {
        if (m_base != NULL)
        {
            delete m_base;
        }
    }

    void setInfraredType(SDKInfraredType type)
    {
        if (m_base != NULL)
        {
            delete m_base;
            m_base = NULL;
        }
        m_infraredType = type;
        getInfraredObject();
    }

private:
    InfraredBase * m_base = NULL;
    SDKInfraredType m_infraredType = SDK_JUGE_INFRARED;
};

#define INFRARED_BASE InfraredChoose::GetSingleton()
#define INFRARED_OBJECT INFRARED_BASE.getInfraredObject()