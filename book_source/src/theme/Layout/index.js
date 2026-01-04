// // src/theme/Root.js
// import { AuthProvider } from '@site/src/context/AuthContext'
// import Chatbot from '@site/src/components/Chatbot';

// export default function Root({ children }) {
//   return (
//     <AuthProvider>
//       {children}
//       <Chatbot />  
//     </AuthProvider>
//   );
// }



// // // src/theme/Layout/index.js
import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import { AuthProvider } from '@site/src/context/AuthContext';
import Chatbot from '@site/src/components/Chatbot';

export default function Layout(props) {
  return (
    <AuthProvider>
      <OriginalLayout {...props} />
      <Chatbot />
    </AuthProvider>
  );
}