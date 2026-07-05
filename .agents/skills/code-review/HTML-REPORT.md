# HTML Report Format

The code review is rendered as a single self-contained HTML file in the OS temp dir. Tailwind comes from CDN; no other dependencies.

## Scaffold

```html
<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Code review — {{branch}} · {{repo}}</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <style>
      /* severity dots, filter tags, expandable detail panels */
      .sev-critical { background: #dc2626; }
      .sev-warning  { background: #d97706; }
      .sev-note     { background: #2563eb; }
      .finding-row  { cursor: pointer; }
      .detail-panel { display: none; }
      .detail-panel.open { display: table-row; }
    </style>
  </head>
  <body class="bg-stone-50 text-slate-900 font-sans">
    <main class="max-w-6xl mx-auto px-6 py-10 space-y-8">
      <header>...</header>
      <section id="summary">...</section>
      <section id="findings">...</section>
      <section id="top-issues">...</section>
    </main>
    <script>
      /* filter + expand/collapse logic */
    </script>
  </body>
</html>
```

## Header

Branch name, review scope (e.g. "5 commits ahead of origin/main + working tree changes"), timestamp, total finding count with a severity breakdown.

Compact, no introduction paragraph.

```html
<header class="space-y-1">
  <h1 class="text-2xl font-bold tracking-tight">Code Review</h1>
  <p class="text-sm text-slate-500">
    <span class="font-mono">{{branch}}</span> —
    {{N}} commits ahead of <span class="font-mono">origin/main</span> + working tree
  </p>
  <p class="text-sm text-slate-400">{{timestamp}}</p>
  <div class="flex gap-4 mt-3">
    <span class="flex items-center gap-1 text-sm font-medium">
      <span class="w-2.5 h-2.5 rounded-full sev-critical"></span> {{critical_count}} Critical
    </span>
    <span class="flex items-center gap-1 text-sm font-medium">
      <span class="w-2.5 h-2.5 rounded-full sev-warning"></span> {{warning_count}} Warning
    </span>
    <span class="flex items-center gap-1 text-sm font-medium">
      <span class="w-2.5 h-2.5 rounded-full sev-note"></span> {{note_count}} Note
    </span>
  </div>
</header>
```

## Summary Cards

One card per review dimension that had findings. Each card shows the dimension name and a mini bar chart (horizontal stacked bar of critical/warning/note counts). If a dimension was clean, show a green checkmark card.

```html
<section id="summary" class="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-6 gap-3">
  <div class="rounded-lg border border-slate-200 bg-white p-4">
    <h3 class="text-xs font-semibold uppercase tracking-wider text-slate-500 mb-2">Safety</h3>
    <div class="flex h-2 rounded-full overflow-hidden bg-slate-100">
      <div class="sev-critical" style="width:33%"></div>
      <div class="sev-warning" style="width:50%"></div>
      <div class="sev-note" style="width:17%"></div>
    </div>
    <div class="flex justify-between mt-2 text-xs text-slate-500">
      <span class="text-red-600 font-semibold">2</span>
      <span class="text-amber-600 font-semibold">3</span>
      <span class="text-blue-600 font-semibold">1</span>
    </div>
  </div>
  <!-- repeat per dimension -->
</section>
```

## Findings Table

Sortable (click column headers), filterable (severity toggle pills, category dropdown, file search input). Columns: File, Line, Sev, Category, Title (compact view).

```html
<section id="findings" class="space-y-4">
  <div class="flex flex-wrap gap-2 items-center">
    <!-- filter pills -->
    <button class="px-3 py-1 text-xs font-medium rounded-full border sev-critical text-white">Critical</button>
    <button class="px-3 py-1 text-xs font-medium rounded-full border border-red-300 text-red-700 bg-white">Warning</button>
    <button class="px-3 py-1 text-xs font-medium rounded-full border border-blue-300 text-blue-700 bg-white">Note</button>
    <select class="ml-auto text-sm border border-slate-300 rounded px-2 py-1">...</select>
    <input type="text" placeholder="Filter by file…" class="text-sm border border-slate-300 rounded px-2 py-1 w-48" />
  </div>

  <div class="overflow-x-auto rounded-lg border border-slate-200 bg-white">
    <table class="w-full text-sm">
      <thead>
        <tr class="bg-slate-50 text-left text-xs uppercase tracking-wider text-slate-500">
          <th class="px-4 py-3 cursor-pointer select-none">File ▾</th>
          <th class="px-4 py-3 cursor-pointer select-none w-16">Line</th>
          <th class="px-4 py-3 cursor-pointer select-none w-20">Sev</th>
          <th class="px-4 py-3 cursor-pointer select-none w-32">Category</th>
          <th class="px-4 py-3">Title</th>
        </tr>
      </thead>
      <tbody>
        <tr class="finding-row border-t border-slate-100 hover:bg-slate-50" onclick="this.nextElementSibling.classList.toggle('open')">
          <td class="px-4 py-2.5 font-mono text-xs">{{file}}</td>
          <td class="px-4 py-2.5 text-xs text-slate-500">{{line}}</td>
          <td class="px-4 py-2.5"><span class="inline-block w-2 h-2 rounded-full sev-{{severity}}"></span> {{severity}}</td>
          <td class="px-4 py-2.5 text-xs text-slate-500">{{category}}</td>
          <td class="px-4 py-2.5 font-medium">{{title}}</td>
        </tr>
        <tr class="detail-panel border-t border-slate-100 bg-slate-50">
          <td colspan="5" class="px-6 py-4">
            <p class="text-sm text-slate-700 whitespace-pre-line">{{detail}}</p>
            <pre class="mt-2 bg-slate-900 text-slate-100 text-xs p-3 rounded overflow-x-auto"><code>{{snippet}}</code></pre>
          </td>
        </tr>
      </tbody>
    </table>
  </div>
</section>
```

### Filter behaviour

Vanilla JS, no framework. Clicking a severity pill toggles that severity on/off. The select filters by category. The text input filters by file path substring. All filters combine with AND. If no rows match, show "No findings match — try adjusting filters."

### Sort behaviour

Click a column header to sort ascending; click again for descending. An arrow indicator shows current sort direction. Default sort: severity (critical first, then warning, then note), then file path alphabetical.

### Expand behaviour

Click a row to toggle its detail panel. Detail shows the full explanation and a syntax-highlighted code snippet (using `<pre><code>` with manual token colouring — no highlight.js CDN, keep it light).

## Top Issues

The 3 most actionable findings. Each gets a bordered card:

```html
<section id="top-issues" class="space-y-4">
  <h2 class="text-lg font-semibold">Top Issues</h2>
  {{#each top_issues}}
  <div class="rounded-lg border border-slate-200 bg-white p-5">
    <div class="flex items-start gap-3">
      <span class="mt-0.5 sev-{{severity}} text-white text-xs font-bold px-2 py-0.5 rounded">{{severity}}</span>
      <div>
        <h3 class="font-semibold">{{title}}</h3>
        <p class="text-sm text-slate-500 mt-1"><span class="font-mono">{{file}}:{{line}}</span></p>
        <p class="text-sm text-slate-700 mt-2">{{summary}}</p>
        <div class="mt-3 bg-slate-50 rounded p-3 text-sm">
          <span class="font-semibold text-slate-600">Fix:</span> {{fix_suggestion}}
        </div>
      </div>
    </div>
  </div>
  {{/each}}
</section>
```

## Style guidance

- Editorial, not dashboard. Generous whitespace. Stone/slate palette.
- One accent per severity (red/amber/blue). No other colours.
- `font-mono` for file paths, line numbers, code snippets.
- No icons, no emojis in the report. Let the data speak.
- The only script is the Tailwind CDN plus the ~30 lines of filter/sort/expand JS. No framework, no runtime.
- Keep the detail panel code snippets to ≤20 lines — just enough context to see the issue. Reference line numbers.
- Scroll the table vertically if it exceeds viewport height; keep the header fixed.

## Tone

Plain English, direct. No hedging ("seems like", "maybe", "perhaps", "could be worth considering"). Each finding states what's wrong and what to do.

**Good titles:**
- "Unsafety comment missing for PAC register write"
- "Lock acquired across .await point in distance control"
- "`unwrap()` on encoder read — silent panic path"
- "New term 'obstacle fusion' missing from glossary"

**Bad titles:**
- "Potential issue with unsafe code"
- "Documentation could be improved"
- "Might want to consider adding a glossary entry"
