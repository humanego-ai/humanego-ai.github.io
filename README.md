# HumanEgo Project Page

This folder is a self-contained static site that can be published to
`https://humanego-ai.github.io` via GitHub Pages.

## Local preview

Just open `index.html` in a browser, or run:

```bash
cd website
python3 -m http.server 8000
# then open http://localhost:8000
```

## 1. Prepare the figures

The page references PNGs in `static/images/`. Convert the paper PDFs to PNG with:

```bash
cd website
bash convert_figs.sh
```

(Requires `pdftoppm`, which ships with `poppler-utils`. On macOS: `brew install poppler`; on Ubuntu: `sudo apt install poppler-utils`.)

The script copies:

| Target                         | Source PDF                          |
|--------------------------------|-------------------------------------|
| `static/images/teaser.png`     | `../paper/figs/teaser.pdf`          |
| `static/images/architecture.png` | `../paper/figs/architecture.pdf`  |
| `static/images/tasks.png`      | `../paper/figs/tasks.pdf`           |
| `static/images/baselines.png`  | `../paper/figs/baselines.pdf`       |

Feel free to add more figures, just drop them into `static/images/` and reference
them in `index.html`.

## 2. Create the GitHub org + repo

1. Go to <https://github.com/organizations/new> and create a free organization
   named **`humanego-ai`** (confirm it is available first at
   <https://github.com/humanego-ai>).
2. Inside that org, create a new **public** repository named exactly
   **`humanego-ai.github.io`** (the repo name must match this pattern so GitHub
   serves it at the root URL).
3. Initialize with no README / license / .gitignore (we'll push from here).

## 3. Publish this folder

Because this `website/` folder already lives inside the AriaMimic repo, you
have two deploy options. Pick one.

### Option A — Standalone copy (simplest)

Copy the folder out of AriaMimic and publish it as its own repo:

```bash
cp -r /path/to/AriaMimic/website ~/humanego-site
cd ~/humanego-site
git init -b main
git add .
git commit -m "initial project page"
git remote add origin git@github.com:humanego-ai/humanego-ai.github.io.git
git push -u origin main
```

Later edits live in `~/humanego-site` and you `git push` from there.

### Option B — Git subtree from the AriaMimic repo

If you want to keep editing the site inside AriaMimic (so the source is
backed up with the paper) **and** mirror it out to the Pages repo:

```bash
cd /path/to/AriaMimic
git remote add humanego git@github.com:humanego-ai/humanego-ai.github.io.git
git subtree push --prefix=website humanego main
```

After further edits to `website/`, commit normally to AriaMimic, then re-run
the subtree push:

```bash
git add website && git commit -m "update project page"
git subtree push --prefix=website humanego main
```

---

Whichever option you pick, GitHub Pages publishes automatically from `main`.
Within a minute or two the site will be live at:

**<https://humanego-ai.github.io>**

You can verify/override the Pages settings at
`Settings` → `Pages` → Source: *Deploy from a branch*, branch: `main` / `/ (root)`.

## 4. Customize

Search `index.html` for these placeholders and replace:

- `<iframe src="https://www.youtube.com/embed/dQw4w9WgXcQ"` &rarr; your real video
  embed URL (or swap the iframe for a local `<video>` tag that loads
  `static/videos/main_video.mp4`).
- `Anonymous Author 1..6` &rarr; real author list once the camera-ready version
  is public.
- Paper / arXiv buttons currently marked `class="disabled"` &rarr; remove
  `class="disabled"` and fill in the real `href`.
- BibTeX block &rarr; update with the final citation.

## 5. (Optional) Custom domain later

If you eventually buy a domain (e.g., `humanego.ai`, or a personal domain like
`botaoleo.com`), you only need to:

1. `Settings` → `Pages` → Custom domain &rarr; enter the domain.
2. At your DNS provider, add four A records pointing to
   `185.199.108.153`, `185.199.109.153`, `185.199.110.153`, `185.199.111.153`,
   plus a CNAME `www` &rarr; `humanego-ai.github.io`.
3. Wait for propagation (~5 min) and enable *Enforce HTTPS*.

The old `humanego-ai.github.io` URL continues to 301-redirect to the custom
domain, so no links break.
