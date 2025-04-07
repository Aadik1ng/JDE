import React from 'react';
import { useDispatch, useSelector } from 'react-redux';
import styled from 'styled-components';
import { setFilter, setSort } from '../store/taskSlice';

const FilterContainer = styled.div`
  display: flex;
  flex-direction: column;
  gap: 1rem;
`;

const FilterGroup = styled.div`
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
`;

const Label = styled.label`
  font-size: 0.9rem;
  color: #495057;
`;

const Select = styled.select`
  padding: 0.5rem;
  border: 1px solid #ced4da;
  border-radius: 4px;
  font-size: 0.9rem;
`;

const Input = styled.input`
  padding: 0.5rem;
  border: 1px solid #ced4da;
  border-radius: 4px;
  font-size: 0.9rem;
`;

const SearchInput = styled(Input)`
  width: 100%;
`;

const SortGroup = styled(FilterGroup)`
  margin-top: 1rem;
  padding-top: 1rem;
  border-top: 1px solid #dee2e6;
`;

const TaskFilters = () => {
  const dispatch = useDispatch();
  const { categories, filter, sortBy, sortOrder } = useSelector(state => state.tasks);

  const handleFilterChange = (e) => {
    const { name, value } = e.target;
    dispatch(setFilter({ [name]: value }));
  };

  const handleSearchChange = (e) => {
    dispatch(setFilter({ searchQuery: e.target.value }));
  };

  const handleSortChange = (e) => {
    const { name, value } = e.target;
    dispatch(setSort({ [name]: value }));
  };

  return (
    <FilterContainer>
      <FilterGroup>
        <Label htmlFor="search">Search Tasks</Label>
        <SearchInput
          type="text"
          id="search"
          placeholder="Search by title..."
          value={filter.searchQuery}
          onChange={handleSearchChange}
        />
      </FilterGroup>

      <FilterGroup>
        <Label htmlFor="statusFilter">Status</Label>
        <Select
          id="statusFilter"
          name="status"
          value={filter.status}
          onChange={handleFilterChange}
        >
          <option value="all">All Tasks</option>
          <option value="incomplete">Incomplete</option>
          <option value="completed">Completed</option>
        </Select>
      </FilterGroup>

      <FilterGroup>
        <Label htmlFor="categoryFilter">Category</Label>
        <Select
          id="categoryFilter"
          name="category"
          value={filter.category}
          onChange={handleFilterChange}
        >
          <option value="all">All Categories</option>
          {categories.map(category => (
            <option key={category} value={category}>
              {category.charAt(0).toUpperCase() + category.slice(1)}
            </option>
          ))}
        </Select>
      </FilterGroup>

      <FilterGroup>
        <Label htmlFor="priorityFilter">Priority</Label>
        <Select
          id="priorityFilter"
          name="priority"
          value={filter.priority}
          onChange={handleFilterChange}
        >
          <option value="all">All Priorities</option>
          <option value="low">Low</option>
          <option value="medium">Medium</option>
          <option value="high">High</option>
        </Select>
      </FilterGroup>

      <SortGroup>
        <Label htmlFor="sortBy">Sort By</Label>
        <Select
          id="sortBy"
          name="sortBy"
          value={sortBy}
          onChange={handleSortChange}
        >
          <option value="priority">Priority</option>
          <option value="dueDate">Due Date</option>
          <option value="createdAt">Created Date</option>
        </Select>

        <Label htmlFor="sortOrder">Sort Order</Label>
        <Select
          id="sortOrder"
          name="sortOrder"
          value={sortOrder}
          onChange={handleSortChange}
        >
          <option value="desc">Descending</option>
          <option value="asc">Ascending</option>
        </Select>
      </SortGroup>
    </FilterContainer>
  );
};

export default TaskFilters; 