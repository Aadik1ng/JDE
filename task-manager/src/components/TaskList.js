import React from 'react';
import { useSelector, useDispatch } from 'react-redux';
import styled from 'styled-components';
import { DragDropContext, Droppable, Draggable } from 'react-beautiful-dnd';
import { toggleTask, deleteTask, reorderTasks } from '../store/taskSlice';
import { selectFilteredTasks } from '../store/taskSlice';

const List = styled.ul`
  list-style: none;
  padding: 0;
  margin: 0;
`;

const ListItem = styled.li`
  background: white;
  padding: 1rem;
  margin-bottom: 0.5rem;
  border-radius: 4px;
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
  display: flex;
  justify-content: space-between;
  align-items: center;
  transition: transform 0.2s, box-shadow 0.2s;
  border-left: 4px solid ${props => {
    switch (props.priority) {
      case 'high': return '#dc3545';
      case 'medium': return '#ffc107';
      case 'low': return '#28a745';
      default: return '#6c757d';
    }
  }};

  &:hover {
    transform: translateX(4px);
    box-shadow: 0 2px 5px rgba(0, 0, 0, 0.2);
  }
`;

const TaskInfo = styled.div`
  flex: 1;
  display: flex;
  align-items: center;
  gap: 1rem;
`;

const Checkbox = styled.input`
  width: 18px;
  height: 18px;
  cursor: pointer;
`;

const TaskContent = styled.div`
  flex: 1;
`;

const TaskTitle = styled.h3`
  margin: 0;
  color: #2c3e50;
  text-decoration: ${props => props.completed ? 'line-through' : 'none'};
  color: ${props => props.completed ? '#6c757d' : '#2c3e50'};
`;

const TaskMeta = styled.div`
  font-size: 0.9rem;
  color: #666;
  margin-top: 0.5rem;
  display: flex;
  gap: 1rem;
  flex-wrap: wrap;
`;

const Badge = styled.span`
  background: ${props => {
    switch (props.type) {
      case 'category': return '#e9ecef';
      case 'priority': return props.priority === 'high' ? '#dc3545' : 
                        props.priority === 'medium' ? '#ffc107' : '#28a745';
      case 'dueDate': return '#17a2b8';
      default: return '#6c757d';
    }
  }};
  color: ${props => props.type === 'priority' ? 'white' : '#495057'};
  padding: 0.25rem 0.5rem;
  border-radius: 4px;
  font-size: 0.8rem;
`;

const DeleteButton = styled.button`
  background: none;
  border: none;
  color: #dc3545;
  cursor: pointer;
  padding: 0.5rem;
  font-size: 1.2rem;
  opacity: 0;
  transition: opacity 0.2s;

  ${ListItem}:hover & {
    opacity: 1;
  }

  &:hover {
    color: #c82333;
  }
`;

const EmptyState = styled.p`
  text-align: center;
  color: #6c757d;
  padding: 2rem;
  background: #f8f9fa;
  border-radius: 4px;
`;

const TaskList = () => {
  const dispatch = useDispatch();
  const tasks = useSelector(selectFilteredTasks);

  const handleDragEnd = (result) => {
    if (!result.destination) return;
    
    dispatch(reorderTasks({
      sourceIndex: result.source.index,
      destinationIndex: result.destination.index
    }));
  };

  const handleToggleTask = (taskId) => {
    dispatch(toggleTask(taskId));
  };

  const handleDeleteTask = (taskId) => {
    dispatch(deleteTask(taskId));
  };

  if (tasks.length === 0) {
    return <EmptyState>No tasks found. Add a task to get started!</EmptyState>;
  }

  return (
    <DragDropContext onDragEnd={handleDragEnd}>
      <Droppable droppableId="tasks">
        {(provided) => (
          <List {...provided.droppableProps} ref={provided.innerRef}>
            {tasks.map((task, index) => (
              <Draggable key={task.id} draggableId={task.id.toString()} index={index}>
                {(provided, snapshot) => (
                  <ListItem
                    ref={provided.innerRef}
                    {...provided.draggableProps}
                    {...provided.dragHandleProps}
                    priority={task.priority}
                    style={{
                      ...provided.draggableProps.style,
                      background: snapshot.isDragging ? '#f8f9fa' : 'white'
                    }}
                  >
                    <TaskInfo>
                      <Checkbox
                        type="checkbox"
                        checked={task.completed}
                        onChange={() => handleToggleTask(task.id)}
                      />
                      <TaskContent>
                        <TaskTitle completed={task.completed}>{task.title}</TaskTitle>
                        <TaskMeta>
                          <Badge type="category">{task.category}</Badge>
                          <Badge type="priority" priority={task.priority}>
                            {task.priority}
                          </Badge>
                          {task.dueDate && (
                            <Badge type="dueDate">
                              Due: {new Date(task.dueDate).toLocaleDateString()}
                            </Badge>
                          )}
                        </TaskMeta>
                      </TaskContent>
                    </TaskInfo>
                    <DeleteButton onClick={() => handleDeleteTask(task.id)}>
                      ×
                    </DeleteButton>
                  </ListItem>
                )}
              </Draggable>
            ))}
            {provided.placeholder}
          </List>
        )}
      </Droppable>
    </DragDropContext>
  );
};

export default TaskList; 