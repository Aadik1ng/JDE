import { configureStore } from '@reduxjs/toolkit';
import taskReducer from './taskSlice';
import userReducer from './userSlice';

// Load state from localStorage
const loadState = () => {
  try {
    const serializedState = localStorage.getItem('state');
    if (serializedState === null) {
      return undefined;
    }
    return JSON.parse(serializedState);
  } catch (err) {
    return undefined;
  }
};

// Save state to localStorage
const saveState = (state) => {
  try {
    const serializedState = JSON.stringify(state);
    localStorage.setItem('state', serializedState);
  } catch (err) {
    // Ignore write errors
  }
};

const store = configureStore({
  reducer: {
    tasks: taskReducer,
    users: userReducer
  },
  preloadedState: loadState(),
});

// Subscribe to store changes to save state
store.subscribe(() => {
  saveState(store.getState());
});

export default store; 