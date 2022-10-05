#include "MessageNotifier.h"


MessageNotifier* MessageNotifier::m_inst = NULL;
QMutex MessageNotifier::m_mutex;

MessageNotifier* MessageNotifier::getInstance()
{
	m_mutex.lock();
	if (m_inst == NULL)
	{
		m_inst = new MessageNotifier;
	}
	m_mutex.unlock();

	return m_inst;
}

MessageNotifier::MessageNotifier()
{
}


MessageNotifier::~MessageNotifier()
{
}
