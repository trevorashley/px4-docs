# PX4 Autopilot Notes

A personal [mdBook](https://rust-lang.github.io/mdBook/) about the PX4 autopilot.

## Build and preview

Install mdBook 0.5.4 if it is not already available (requires Rust and Cargo):

```sh
cargo install mdbook --version 0.5.4 --locked
```

From the repository root, build the site:

```sh
mdbook build
```

The generated site is in `book/`, which is ignored by Git. To preview with live
reload, run:

```sh
mdbook serve --open
```

The local preview is available at <http://localhost:3000>.

## Add documentation

Edit `src/welcome.md`, or create a Markdown file in `src/` and add a link to
`src/SUMMARY.md` so it appears in the book. Change the title and other settings
in `book.toml`.

The book includes [mdBook-pagetoc](https://github.com/JorelAli/mdBook-pagetoc),
which adds a table of headings on the right of each page on wide screens.
Its assets are included in `theme/`; no additional installation is needed.
See `theme/README.md` for the upstream version and template maintenance notes.

## Publish with GitHub Pages

1. In this repository's **Settings → Pages → Build and deployment**, select
   **GitHub Actions** as the source.
2. Commit and push these files to `main`. The **Build and deploy book** workflow
   builds the book and publishes it. You can also run it from the Actions tab.
3. Once deployment succeeds, visit <https://trevorashley.github.io/px4-docs/>.

Pull requests targeting `main` build the book without deploying it. Publishing
does not require committing the generated `book/` directory or creating a
`gh-pages` branch.

The workflow uses GitHub's [Pages deployment actions](https://github.com/actions/starter-workflows/blob/main/pages/mdbook.yml).
If the repository name or hosting path changes, update `site-url` in `book.toml`.
