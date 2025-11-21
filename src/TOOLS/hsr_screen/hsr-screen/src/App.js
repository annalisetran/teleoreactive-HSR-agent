import './App.css';

import { createGlobalStyle } from 'styled-components';
import { BrowserRouter } from 'react-router-dom'
import Router from './Router'
import '@fontsource/roboto';
import './index.css'



const GlobalStyle = createGlobalStyle`
  body {
    margin: 0;
    letter-spacing: .15rem;
    font-family: "Quantico", sans-serif;
    font-weight: 400;
    font-style: normal
  }
`;

function App() {
  return (
    <>
      <GlobalStyle />
      <BrowserRouter>
        <Router/>
      </BrowserRouter>
    </>
  );
}

export default App;
