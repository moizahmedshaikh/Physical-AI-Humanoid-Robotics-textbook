# Skill: modern-auth-ui

## Purpose
Create beautiful, modern, and minimal authentication UI components for Docusaurus projects. Build visually appealing login and signup pages with smooth animations, gradient buttons, and professional design that matches contemporary web standards.

## Capabilities
This skill can:
- Design modern login pages with glassmorphism effects
- Create signup forms with smooth transitions and animations
- Build animated gradient buttons with hover effects
- Implement floating label inputs with micro-animations
- Add loading states with skeleton screens
- Create error/success toast notifications
- Design responsive layouts for mobile and desktop
- Apply dark mode compatible styling
- Add subtle parallax or background effects
- Implement form validation with animated error messages

## How it Performs Actions

### 1. Login Page Design
- **Layout**: Centered card with glassmorphism effect
- **Elements**:
  - Animated logo/title with fade-in
  - Floating label input fields (email, password)
  - Gradient animated submit button
  - "Forgot password?" link with hover effect
  - "Sign up" link with smooth transition
  - Social login buttons

### 2. Signup Page Design
- **Layout**: Centered card with glassmorphism effect
- **Elements**:
  - Animated logo/title with fade-in
  - Floating label input fields (name, email, password, confirm password)
  - Password strength indicator
  - Gradient animated submit button
  - "Already have an account? Sign in" link
  - Terms and conditions checkbox with animation

### 3. Component Creation
- **Floating Input**: Input with animated label that floats on focus
- **Gradient Button**: Animated button with gradient background and hover effects
- **Password Strength Indicator**: Visual feedback for password strength
- **Toast Notifications**: Animated success/error messages
- **Skeleton Loaders**: Animated loading placeholders
- **Auth Layout**: Shared wrapper component with responsive design

## Input Parameters
- Page type: "login", "signup", or "both"
- Theme preference: "light", "dark", or "auto"
- Color scheme: Primary gradient colors
- Animation intensity: "subtle", "moderate", "bold"
- Brand logo URL (optional)
- Custom styling overrides (optional)

## Output
- Complete React component files:
  - `LoginPage.tsx` - Login page component
  - `SignupPage.tsx` - Signup page component
  - `AuthLayout.tsx` - Shared layout wrapper
  - `FloatingInput.tsx` - Reusable input component
  - `GradientButton.tsx` - Animated button component
  - `PasswordStrength.tsx` - Password indicator
- CSS/Styled-components:
  - `authStyles.css` - Authentication page styles
  - Animation keyframes
  - Responsive media queries
- Integration guide for Docusaurus

## Usage Notes
- Compatible with Docusaurus 3.x
- Uses only CSS animations (no heavy libraries)
- Optional: Framer Motion for advanced animations
- Fully responsive (mobile, tablet, desktop)
- Accessible (WCAG 2.1 AA compliant)
- Dark mode compatible
- RTL support for Urdu version

## Design Inspiration
- Modern SaaS login pages (Notion, Linear, Vercel)
- Glassmorphism trend (iOS design language)
- Neumorphism subtle shadows
- Gradient mesh backgrounds
- Micro-interactions and delightful animations

## Integration with Other Skills
- Works with `protected-route-guard` for routing
- Calls `auth-api-builder` endpoints on form submission
- Displays errors from backend validation
- Integrates with `AuthContext` for state management

## Accessibility Features
- Keyboard navigation (Tab, Enter)
- Screen reader labels
- Focus indicators
- Error announcements
- Color contrast compliance
- Touch target sizes (44px minimum)

## Example Implementation
```typescript
// LoginPage.tsx structure
import { useState } from 'react';
import FloatingInput from './FloatingInput';
import GradientButton from './GradientButton';
import './authStyles.css';

export default function LoginPage() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [loading, setLoading] = useState(false);

  return (
    <div className="auth-layout">
      <div className="login-card glassmorphism">
        <div className="animated-logo">
          {/* Logo with fade-in animation */}
        </div>

        <form onSubmit={handleLogin}>
          <FloatingInput
            type="email"
            value={email}
            onChange={setEmail}
            label="Email Address"
            icon="mail"
            required
          />

          <FloatingInput
            type="password"
            value={password}
            onChange={setPassword}
            label="Password"
            icon="lock"
            required
          />

          <GradientButton
            type="submit"
            loading={loading}
            text="Sign In"
            loadingText="Signing in..."
          />
        </form>

        <div className="auth-links">
          <a href="/signup">Don't have an account? Sign up</a>
        </div>
      </div>
    </div>
  );
}
```

## Success Criteria
- UI looks modern and professional
- Animations are smooth (60fps)
- Forms are fully functional
- Mobile experience is excellent
- Loading states are clear
- Errors are user-friendly
- Design matches Docusaurus theme
- Dark mode works perfectly