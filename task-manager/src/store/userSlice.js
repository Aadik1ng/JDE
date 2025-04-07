import { createSlice } from '@reduxjs/toolkit';

const initialState = {
  users: [],
  currentUser: null,
  error: null
};

const userSlice = createSlice({
  name: 'users',
  initialState,
  reducers: {
    registerUser: (state, action) => {
      const { email, password, name } = action.payload;
      
      // Check if user already exists
      if (state.users.some(user => user.email === email)) {
        state.error = 'User already exists';
        return;
      }

      // Create new user with hashed password (in production, use proper hashing)
      const newUser = {
        id: Date.now(),
        email,
        password, // In production, use proper password hashing
        name,
        createdAt: new Date().toISOString()
      };

      state.users.push(newUser);
      state.error = null;
    },
    loginUser: (state, action) => {
      const { email, password } = action.payload;
      const user = state.users.find(u => u.email === email && u.password === password);

      if (user) {
        state.currentUser = user;
        state.error = null;
      } else {
        state.error = 'Invalid email or password';
      }
    },
    logoutUser: (state) => {
      state.currentUser = null;
      state.error = null;
    },
    updateUserProfile: (state, action) => {
      const { name, email } = action.payload;
      const user = state.users.find(u => u.id === state.currentUser.id);

      if (user) {
        user.name = name;
        user.email = email;
        state.currentUser = { ...state.currentUser, name, email };
      }
    },
    clearError: (state) => {
      state.error = null;
    }
  }
});

export const {
  registerUser,
  loginUser,
  logoutUser,
  updateUserProfile,
  clearError
} = userSlice.actions;

export default userSlice.reducer; 