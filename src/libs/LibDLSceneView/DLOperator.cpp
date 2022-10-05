#include <QGraphicsScene>
#include <QUndoStack>
#include <QGraphicsSceneMouseEvent>
#include <iostream>
#include "DLOperator.h"
#include "DLCommands.h"

#include "DLCustomScene.h"

DLOperator::DLOperator(QGraphicsScene *scene):scene_(scene)
{

	undo_stack_=new QUndoStack();
	type_=TYPE_MOVE;
	cmd_=NULL;

}


void DLOperator::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	 
	if(event->button()==Qt::RightButton)
	{
		set_type(TYPE_MOVE);
		return;
	}

	if (type_ != TYPE_SELECT)
	{
		DLCustomScene *pScene = dynamic_cast<DLCustomScene *>(scene_);
		if (pScene)
		{
			emit pScene->SMAPChangedSignal();
		}

	}

	switch(type_)
	{
		case TYPE_SELECT:
		{
			cmd_=new DLSelectCommand(scene_);
			cmd_->mousePressEvent(event);
			break;		
		}
		case TYPE_COORDINATE_TRANSFORM:
		{
			cmd_ = new DLCoordinateTransformCommand(scene_);
			cmd_->mousePressEvent(event);
			break;
		}
		case TYPE_DEL:
		{
			cmd_ = new DLDeleteCommand(scene_);
			cmd_->mousePressEvent(event);
			break;
		}
		case TYPE_ADD_LANDMARK:	
		{
			cmd_ = new DLAddLandmarkCommand(scene_);
			cmd_->mousePressEvent(event);
			break;
		}
		case TYPE_ADD_EDGE:
		{
			cmd_ = new DLAddEdgeCommand(scene_);
			cmd_->mousePressEvent(event);	
			break;
		}
		case TYPE_RELOCATION:
		{
			cmd_ = new DLRelocationCommand(scene_);
			cmd_->mousePressEvent(event);	
			break;
		}
		case TYPE_ADD_DEVICE:
		{
			cmd_ = new DLAddDeviceCommand(scene_);
			cmd_->mousePressEvent(event);
			break;
		}
		case TYPE_ADD_ARC:
		{
			cmd_ = new DLAddArcCommand(scene_);
			cmd_->mousePressEvent(event);
			break;
		}
		case TYPE_ADD_PICTURE:
		{
			cmd_ = new DLAddPictureCommand(scene_);
			cmd_->mousePressEvent(event);
			break;
		}
		case TYPE_ADD_AREA:
		{
			cmd_ = new DLAddPolygonCommand(scene_);
			cmd_->mousePressEvent(event);
			break;
		}
		case TYPE_ADD_LINE:
		{
			cmd_ = new DLAddLineCommand(scene_, -1);
			cmd_->mousePressEvent(event);
			break;
		}
		case TYPE_ADD_VIRTUAL_LINE:
		{
			cmd_ = new DLAddLineCommand(scene_, 1);
			cmd_->mousePressEvent(event);
			break;
		}
		case TYPE_ADD_FORBBIDON_LINE:
		{
			cmd_ = new DLAddLineCommand(scene_, 0);
			cmd_->mousePressEvent(event);
			break;
		}
		case TYPE_ADD_DEVAREA:
		{
			cmd_ = new DLAddDevAreaCommand(scene_);
			cmd_->mousePressEvent(event);
			break;
		}
		case TYPE_ADD_STATION:
		{
			cmd_ = new DLAddSTationAreaCommand(scene_);
			cmd_->mousePressEvent(event);
			break;
		}
		default:
			break; 
	}

}


void DLOperator::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{

	if(event->button()==Qt::RightButton)
	{
		return;
	}

	if(cmd_!=NULL)
	{

		switch(type_)
		{
		case TYPE_SELECT:
		{
			cmd_->mouseMoveEvent(event);
			break;	
		}
		case TYPE_COORDINATE_TRANSFORM:
		{
			cmd_->mouseMoveEvent(event);
			break;
		}
		case TYPE_DEL:
		{
			cmd_->mouseMoveEvent(event);
			break;
		}
		case TYPE_ADD_LANDMARK:	
		{
			cmd_->mouseMoveEvent(event);
			break;
		}
		case TYPE_ADD_EDGE:
		{
			cmd_->mouseMoveEvent(event);
			break;
		}
		case TYPE_RELOCATION:
		{
			cmd_->mouseMoveEvent(event);
			break;
		}
		case TYPE_ADD_DEVICE:
		{
			cmd_->mouseMoveEvent(event);
			break;
		}
		case TYPE_ADD_ARC:
		{
			cmd_->mouseMoveEvent(event);
			break;
		}
		case TYPE_ADD_PICTURE:
		{
			cmd_->mouseMoveEvent(event);
			break;
		}
		case TYPE_ADD_AREA:
		{
			cmd_->mouseMoveEvent(event);
			break;
		}
		case TYPE_ADD_LINE:
		{
			cmd_->mouseMoveEvent(event);
			break;
		}
		case TYPE_ADD_VIRTUAL_LINE:
		{
			cmd_->mouseMoveEvent(event);
			break;
		}
		case TYPE_ADD_FORBBIDON_LINE:
		{
			cmd_->mouseMoveEvent(event);
			break;
		}
		case TYPE_ADD_DEVAREA:
		{
			cmd_->mouseMoveEvent(event);
			break;
		}
		case TYPE_ADD_STATION:
		{
			cmd_->mouseMoveEvent(event);
			break;
		}
		default:
			break;
		}
	}
}


void DLOperator::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{

	if(event->button()==Qt::RightButton)
	{
		return;
	}

	if(cmd_!=NULL)
	{

		switch(type_)
		{
		case TYPE_SELECT:
		{
			cmd_->mouseReleaseEvent(event);
			add_cmd(&cmd_);
			break;				
		}
		case TYPE_COORDINATE_TRANSFORM:
		{
			cmd_->mouseReleaseEvent(event);
			break;
		}
		case TYPE_DEL:
		{
			cmd_->mouseReleaseEvent(event);
			break;
		}
		case TYPE_ADD_LANDMARK:	
		{
			cmd_->mouseReleaseEvent(event);
			add_cmd(&cmd_);
			break;
		}
		case TYPE_ADD_EDGE:
		{
			cmd_->mouseReleaseEvent(event);
			add_cmd(&cmd_);	
			break;
		}
		case TYPE_RELOCATION:
		{
			cmd_->mouseReleaseEvent(event);
			add_cmd(&cmd_);
			break;
		}
		case TYPE_ADD_DEVICE:
		{
			cmd_->mouseReleaseEvent(event);
			add_cmd(&cmd_);
			break;
		}
		case TYPE_ADD_ARC:
		{
			cmd_->mouseReleaseEvent(event);
			add_cmd(&cmd_);
			break;
		}
		case TYPE_ADD_PICTURE:
		{
			cmd_->mouseReleaseEvent(event);
			add_cmd(&cmd_);
			break;
		}
		case TYPE_ADD_AREA:
		{
			cmd_->mouseReleaseEvent(event);
			add_cmd(&cmd_);
			break;
		}
		case TYPE_ADD_LINE:
		{
			cmd_->mouseReleaseEvent(event);
			add_cmd(&cmd_);
			break;
		}
		case TYPE_ADD_VIRTUAL_LINE:
		{
			cmd_->mouseReleaseEvent(event);
			add_cmd(&cmd_);
			break;
		}
		case TYPE_ADD_FORBBIDON_LINE:
		{
			cmd_->mouseReleaseEvent(event);
			add_cmd(&cmd_);
			break;
		}
		case TYPE_ADD_DEVAREA:
		{
			cmd_->mouseReleaseEvent(event);
			add_cmd(&cmd_);
			break;
		}
		case TYPE_ADD_STATION:
		{
			cmd_->mouseReleaseEvent(event);
			add_cmd(&cmd_);
			break;
		}
		default:
			break; 

		}	
	}
}


void DLOperator::set_type(int type)
{

	type_=type;
	switch(type_)
	{
	case TYPE_SELECT:
		break;
	case TYPE_DEL:
		break;		
	case TYPE_COORDINATE_TRANSFORM:
		break;
	case TYPE_MOVE:
		break;
	case TYPE_ADD_LANDMARK:		
		break;
	case TYPE_ADD_EDGE:
		break;
	case TYPE_RELOCATION:
		break;
	case TYPE_ADD_DEVICE:
		break;		
	case TYPE_ADD_ARC:
		break;		
	case TYPE_ADD_PICTURE:
		break;
	case TYPE_ADD_AREA:
		break;
	case TYPE_ADD_LINE:
		break;
	case TYPE_ADD_DEVAREA:
		break;
	case TYPE_ADD_STATION:
		break;
	default:
		break;
	}
}


QUndoStack* DLOperator::undo_stack()
{
	return undo_stack_;
}


void DLOperator::clean()
{
	undo_stack_->clear();
	cmd_=NULL;



}


void DLOperator::RobotRecation(QPointF ptCurPos, double dAngle)
{
	cmd_ = new DLRelocationCommand(scene_);

	DLRelocationCommand *pCmd = dynamic_cast<DLRelocationCommand *>(cmd_);
	if (nullptr != pCmd)
	{
		pCmd->RobotRelcation(ptCurPos, dAngle);
	}
	add_cmd(&cmd_);
}

void DLOperator::add_cmd(DLUndoCommandInterface **cmd)
{
	if(undo_stack_!=NULL)
	{
		if((*cmd)->is_useful()){
			undo_stack_->push((QUndoCommand*)(*cmd));	
		}
		else
		{
			delete (*cmd);
			(*cmd)=NULL;
		}
	}
}

