import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Robotics',
  tagline: 'Bridging digital AI with physical humanoid robots',

  // Set the production url of your site here
  url: 'https://physical-ai-humanoid-robotics-textb-eosin.vercel.app/',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/',  // ✅ IMPORTANT: Add your repo name
  trailingSlash: false,

  projectName: 'Humanoid-Robotics-textbook',
  organizationName: 'moizahmedshaikh',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // ✅ FIXED: Proper i18n configuration
   i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // routeBasePath: 'docs',  
          editUrl: 'https://github.com/moizahmedshaikh/Physical-AI-Humanoid-Robotics-textbook/tree/main/',
        },
        blog: false, 
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Robotics',
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          label: 'Signup',
          position: 'right',
          href:'/signup'
        },
        {
          label: 'Login',
          position: 'right',
          href:'/login'
        },
        {
          href: 'https://github.com/moizahmedshaikh/Physical-AI-Humanoid-Robotics-textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub Issues',
              href: 'https://github.com/moizahmedshaikh/Physical-AI-Humanoid-Robotics-textbook/issues',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Project GitHub',
              href: 'https://github.com/moizahmedshaikh/Physical-AI-Humanoid-Robotics-textbook',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Robotics. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;