import React, { useState } from 'react';
import styled from 'styled-components';
import TaskList from './components/TaskList';
import TaskForm from './components/TaskForm';
import TaskFilters from './components/TaskFilters';
import Login from './components/Login';
import Register from './components/Register';
import { useSelector, useDispatch } from 'react-redux';
import { logoutUser } from './store/userSlice';

const AppContainer = styled.div`
  min-height: 100vh;
  background: #f8f9fa;
`;

const Header = styled.header`
  background: white;
  padding: 1rem 2rem;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  display: flex;
  justify-content: space-between;
  align-items: center;
`;

const Title = styled.h1`
  margin: 0;
  color: #212529;
`;

const UserInfo = styled.div`
  display: flex;
  align-items: center;
  gap: 1rem;
`;

const UserName = styled.span`
  color: #495057;
`;

const LogoutButton = styled.button`
  background: #dc3545;
  color: white;
  border: none;
  padding: 0.5rem 1rem;
  border-radius: 4px;
  cursor: pointer;
  transition: background 0.2s;

  &:hover {
    background: #c82333;
  }
`;

const MainContent = styled.main`
  max-width: 1200px;
  margin: 2rem auto;
  padding: 0 1rem;
  display: grid;
  grid-template-columns: 250px 1fr;
  gap: 2rem;

  @media (max-width: 768px) {
    grid-template-columns: 1fr;
  }
`;

const Sidebar = styled.aside`
  background: white;
  padding: 1rem;
  border-radius: 8px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
`;

const TaskSection = styled.section`
  background: white;
  padding: 1rem;
  border-radius: 8px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
`;

const AppContent = () => {
  const dispatch = useDispatch();
  const { currentUser } = useSelector(state => state.users);

  const handleLogout = () => {
    dispatch(logoutUser());
  };

  return (
    <AppContainer>
      <Header>
        <Title>Task Manager</Title>
        <UserInfo>
          <UserName>Welcome, {currentUser?.name}</UserName>
          <LogoutButton onClick={handleLogout}>Logout</LogoutButton>
        </UserInfo>
      </Header>
      <MainContent>
        <Sidebar>
          <TaskFilters />
        </Sidebar>
        <TaskSection>
          <TaskForm />
          <TaskList />
        </TaskSection>
      </MainContent>
    </AppContainer>
  );
};

const App = () => {
  const [showRegister, setShowRegister] = useState(false);
  const { currentUser } = useSelector(state => state.users);

  if (currentUser) {
    return <AppContent />;
  }

  return showRegister ? (
    <Register onLoginClick={() => setShowRegister(false)} />
  ) : (
    <Login onRegisterClick={() => setShowRegister(true)} />
  );
};

export default App; 