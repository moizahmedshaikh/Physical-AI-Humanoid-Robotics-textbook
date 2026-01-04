**Skill Version**: 1.0  
**Last Updated**: December 2025
**Compatible With**: Docusaurus 3.x, Spec-Kit Plus  
**Language Pair**: English → Urdu (en → ur)

# Skill: urdu-translator

## Purpose

The **urdu-translator** skill automates the translation of Docusaurus book content from English to Urdu for the Physical AI & Humanoid Robotics textbook. This skill streamlines multilingual documentation creation, ensuring consistency and accuracy while maintaining the technical integrity of the content.

## Capabilities

This skill can:

- Identify English Docusaurus markdown content files in the `book_source/docs/` directory
- Generate corresponding Urdu translation files within the Docusaurus i18n structure (`book_source/i18n/ur/`)
- Perform high-quality machine translation from English to Urdu while preserving:
  - Markdown formatting (headings, lists, code blocks, tables, callouts)
  - YAML frontmatter structure
  - Technical terminology accuracy
  - Code snippets (untranslated)
  - File paths and references
- Translate Docusaurus theme and plugin strings (navbar, footer, sidebar labels)
- Handle nested directory structures (e.g., `chapter-01-foundations/`, `chapter-02-perception/`)
- Preserve special Docusaurus components (admonitions, tabs, code blocks with syntax highlighting)

## How it Performs Actions

The urdu-translator skill will perform actions in the following sequence:

### 1. Pre-Translation Setup

- Verify Docusaurus i18n configuration in `book_source/docusaurus.config.ts`
- Confirm Urdu locale (`ur`) is properly configured
- Check if `npm run write-translations -- --locale ur` has been executed
- Validate the existence of translation JSON files

### 2. Identify Content to Translate

- Scan `book_source/docs/` for all `.md` and `.mdx` files
- Identify files including:
  - Main documentation files (`overview.md`, `introduction.md`)
  - Chapter directories (`chapter-01-foundations/`, `chapter-02-perception/`, etc.)
  - Lesson files within chapters
  - Any nested subdirectories
- Generate a translation manifest with file paths and metadata
- Identify theme/plugin translation files:
  - `book_source/i18n/ur/code.json`
  - `book_source/i18n/ur/docusaurus-theme-classic/navbar.json`
  - `book_source/i18n/ur/docusaurus-theme-classic/footer.json`
  - `book_source/i18n/ur/docusaurus-plugin-content-docs/current.json`

### 3. Translate Markdown Content

For each English markdown file:

- **Create Directory Structure**: Mirror the original structure under `book_source/i18n/ur/docusaurus-plugin-content-docs/current/`
- **Parse Content**: Separate frontmatter, content, and special components
- **Translate Systematically**:
  - **Frontmatter**: Translate only values (title, description, sidebar_label), keep keys unchanged
  - **Headings**: Translate all heading text
  - **Body Text**: Translate paragraphs while preserving technical terms
  - **Code Blocks**: Keep code unchanged, translate comments if present
  - **Links**: Preserve URLs, translate link text
  - **Callouts/Admonitions**: Translate content, keep component syntax
  - **Tables**: Translate cell content, preserve structure
- **Preserve**:
  - All markdown syntax (`#`, `*`, `-`, `>`, etc.)
  - Code fence markers (` ``` `)
  - Image paths and URLs
  - Component imports
  - Mathematical notation
  - File references

### 4. Handle Technical Content

- **Technical Terms**: Maintain English terms with Urdu explanation in parentheses
  - Example: "Embodied Intelligence (مجسم ذہانت)"
- **Acronyms**: Keep English acronyms, provide Urdu expansion
  - Example: "AI (مصنوعی ذہانت - Artificial Intelligence)"
- **Code Variables**: Never translate variable names, function names, or keywords
- **API References**: Keep API endpoints, method names untranslated

### 5. Translate Theme and Plugin Strings

- Read JSON translation files from `book_source/i18n/ur/`
- Translate UI elements:
  - Navbar items (Home, Docs, About)
  - Footer text
  - Sidebar labels
  - Search placeholders
  - Button text
  - Error messages
- Maintain JSON structure and keys
- Ensure proper UTF-8 encoding for Urdu text

### 6. Quality Assurance

- Verify all markdown files have corresponding Urdu versions
- Check for:
  - Broken internal links
  - Missing frontmatter fields
  - Malformed markdown syntax
  - Encoding issues
  - Untranslated content blocks
- Generate a translation coverage report

### 7. Local Testing Guidance

Provide step-by-step testing instructions:

```bash
# Navigate to docs directory
cd book_source

# Start dev server with Urdu locale
npm run start -- --locale ur

# Build for production (test)
npm run build

# Serve built site
npm run serve
```

**Testing Checklist**:

- ✅ Urdu content displays correctly
- ✅ Language switcher works (if implemented)
- ✅ Sidebar navigation is translated
- ✅ Code blocks render properly
- ✅ Internal links function correctly
- ✅ Search works with Urdu content
- ✅ Responsive design maintained
- ✅ RTL (Right-to-Left) layout applied correctly

### 8. Deployment Preparation

- Confirm all translation files are committed
- Update `docusaurus.config.ts` if needed
- Verify build process includes Urdu locale:
  ```bash
  npm run build
  ```
- Ensure GitHub Pages deployment script handles multilingual build
- Provide deployment command:
  ```bash
  npm run deploy
  ```

## Input

### Required

- **Docusaurus project path**: `book_source/`
- **Target locale**: `ur` (Urdu)
- **Source locale**: `en` (English)

### Optional

- Specific files/chapters to translate (for incremental translation)
- Translation API preference (Claude, Gemini, or custom)
- Glossary of technical terms with approved Urdu translations
- Priority order for content translation

## Output

### Translated Files

- Complete Urdu markdown files in `book_source/i18n/ur/docusaurus-plugin-content-docs/current/`
- Translated theme JSON files in `book_source/i18n/ur/docusaurus-theme-classic/`

### Translation Report (Markdown format)

- Total files translated
- Word count (English vs Urdu)
- Files with issues/warnings
- Technical terms glossary used
- Translation coverage percentage

### Testing Instructions

- Command-line instructions for local testing
- Key areas to verify manually
- Known limitations or issues

### Deployment Checklist

- Pre-deployment verification steps
- Build and deployment commands
- Post-deployment validation

## Usage Notes

### Prerequisites

Docusaurus i18n must be configured in `book_source/docusaurus.config.ts`:

```typescript
i18n: {
  defaultLocale: 'en',
  locales: ['en', 'ur'],
  localeConfigs: {
    en: { label: 'English' },
    ur: { label: 'اردو', direction: 'rtl' }
  }
}
```

- Run `npm run write-translations -- --locale ur` before using this skill
- Ensure proper UTF-8 encoding support in the development environment

### Translation Quality

- Uses AI-powered translation (Claude/Gemini) for initial pass
- Technical accuracy prioritized over literal translation
- Urdu scientific/technical terminology standardized
- **Manual review recommended** for:
  - Chapter introductions
  - Complex technical concepts
  - Mathematical explanations
  - Safety-critical content

### Limitations

- Does not translate embedded images or diagrams
- Cannot localize external resources (YouTube videos, external links)
- May require manual adjustment for culturally-specific examples
- RTL layout styling must be handled separately in CSS

### Best Practices

- Translate in chapter-wise batches for easier review
- Maintain a glossary of technical terms for consistency
- Test thoroughly after each major translation batch
- Keep English version as source of truth
- Document any translation decisions or ambiguities

### File Structure Reference

```
book_source/
├── docs/                          # English content
│   ├── overview.md
│   ├── chapter-01-foundations/
│   ├── chapter-02-perception/
│   └── ...
├── i18n/
│   └── ur/                        # Urdu translations
│       ├── code.json
│       ├── docusaurus-theme-classic/
│       │   ├── navbar.json
│       │   └── footer.json
│       └── docusaurus-plugin-content-docs/
│           └── current/           # Translated markdown files
│               ├── overview.md
│               ├── chapter-01-foundations/
│               └── ...
└── docusaurus.config.ts
```

### Maintenance

- Update translations when English content changes
- Version control translation files separately if needed
- Consider using translation memory for future updates
- Document any custom translation rules or exceptions

---