import { createSlice, createSelector } from '@reduxjs/toolkit';

const initialState = {
  items: [],
  filter: 'all',
  categories: ['personal', 'work', 'shopping'],
  sort: 'priority'
};

const taskSlice = createSlice({
  name: 'tasks',
  initialState,
  reducers: {
    addTask: (state, action) => {
      state.items.push({
        ...action.payload,
        id: Date.now(),
        completed: false,
        createdAt: new Date().toISOString()
      });
    },
    toggleTask: (state, action) => {
      const task = state.items.find(t => t.id === action.payload);
      if (task) {
        task.completed = !task.completed;
      }
    },
    deleteTask: (state, action) => {
      state.items = state.items.filter(task => task.id !== action.payload);
    },
    updateTask: (state, action) => {
      const { id, updates } = action.payload;
      const task = state.items.find(t => t.id === id);
      if (task) {
        Object.assign(task, updates);
      }
    },
    setFilter: (state, action) => {
      state.filter = action.payload;
    },
    addCategory: (state, action) => {
      if (!state.categories.includes(action.payload)) {
        state.categories.push(action.payload);
      }
    },
    reorderTasks: (state, action) => {
      const { sourceIndex, destinationIndex } = action.payload;
      const [removed] = state.items.splice(sourceIndex, 1);
      state.items.splice(destinationIndex, 0, removed);
    },
    setSort: (state, action) => {
      state.sort = action.payload;
    },
    clearTasks: (state) => {
      state.items = [];
    }
  }
});

export const {
  addTask,
  toggleTask,
  deleteTask,
  updateTask,
  setFilter,
  addCategory,
  reorderTasks,
  setSort,
  clearTasks
} = taskSlice.actions;

// Selectors
export const selectFilteredTasks = createSelector(
  [
    state => state.tasks.items,
    state => state.tasks.filter,
    state => state.tasks.categories,
    state => state.tasks.sort,
    state => state.users.currentUser
  ],
  (items, filter, categories, sort, currentUser) => {
    // Filter by user
    let filteredItems = items.filter(task => task.userId === currentUser?.id);

    // Apply category filter
    if (filter !== 'all') {
      filteredItems = filteredItems.filter(task => task.category === filter);
    }

    // Apply status filter
    if (filter === 'completed') {
      filteredItems = filteredItems.filter(task => task.completed);
    } else if (filter === 'active') {
      filteredItems = filteredItems.filter(task => !task.completed);
    }

    // Apply sorting
    switch (sort) {
      case 'priority':
        const priorityOrder = { high: 0, medium: 1, low: 2 };
        filteredItems.sort((a, b) => priorityOrder[a.priority] - priorityOrder[b.priority]);
        break;
      case 'dueDate':
        filteredItems.sort((a, b) => new Date(a.dueDate) - new Date(b.dueDate));
        break;
      case 'createdAt':
        filteredItems.sort((a, b) => new Date(b.createdAt) - new Date(a.createdAt));
        break;
      default:
        break;
    }

    return filteredItems;
  }
);

export default taskSlice.reducer; 