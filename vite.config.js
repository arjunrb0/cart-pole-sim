import { defineConfig } from "vite";

// GitHub Pages serves this project from a sub-path:
//   https://arjunrb0.github.io/cart-pole-sim/
// so every built asset URL needs that prefix. `npm run dev` keeps serving
// from the root at localhost:5173/.
//
// Vite reports command "serve" for BOTH the dev server and `vite preview`,
// so the preview needs `isPreview` too; otherwise it serves the build at "/"
// and can't find the prefixed assets, unlike the real site.
export default defineConfig(({ command, isPreview }) => ({
  base: command === "build" || isPreview ? "/cart-pole-sim/" : "/",
}));
